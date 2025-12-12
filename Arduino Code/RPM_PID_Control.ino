#include <math.h>

// ========== Motor pin assignments ==========
// LEFT motor is on channel B: IN1=6, IN2=7, PWM=10
const int LEFT_IN1  = 6;
const int LEFT_IN2  = 7;
const int LEFT_PWM  = 10;

// RIGHT motor is on channel A: IN1=13, IN2=12, PWM=11
const int RIGHT_IN1 = 13;
const int RIGHT_IN2 = 12;
const int RIGHT_PWM = 11;

// ========== Encoder pins ==========
// Left encoder on the LEFT wheel shaft
const int ENC_LEFT_A   = 21;
const int ENC_LEFT_B   = 20;
// Right encoder on the RIGHT wheel shaft
const int ENC_RIGHT_A  = 19;
const int ENC_RIGHT_B  = 18;

// ========== Encoder / wheel constants ==========
const float TICKS_PER_REV      = 1920.0;
const float WHEEL_DIAMETER_MM  = 62.0;
const float WHEEL_CIRCUM_MM    = PI * WHEEL_DIAMETER_MM;
const float DIST_PER_TICK_MM   = WHEEL_CIRCUM_MM / TICKS_PER_REV;

// Wheelbase (distance between wheel contact points, mm)
// Measure and tweak this for more accurate heading
const float WHEEL_BASE_MM      = 150.0;

// ========== Control constants ==========
const int   MIN_PWM_MOVING     = 20;
const int   MAX_PWM            = 255;

// ========== Encoder state ==========
volatile long encLeftCount  = 0;
volatile long encRightCount = 0;

// For PID speed measurement
long prevLeftTicks  = 0;
long prevRightTicks = 0;

float measRPM_L = 0.0;
float measRPM_R = 0.0;

// ========== Motion / PID state ==========
//  1 = forward, -1 = backward, 0 = stop
int moveDir = 0;

// Starting target RPM (same both wheels)
float targetRPM_L = 15.0;
float targetRPM_R = 15.0;

// PID gains (P-only, as per your tuning)
float Kp_L = 1.0, Ki_L = 0.02, Kd_L = 0.01;
float Kp_R = 1.0, Ki_R = 0.01, Kd_R = 0.01;

float integral_L = 0.0, prevError_L = 0.0;
float integral_R = 0.0, prevError_R = 0.0;

// PWM outputs (LEFT / RIGHT wheels)
int pwmLeft  = 0;
int pwmRight = 0;

// Timing
unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 100;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 1000;

// ========== Odometry state ==========
float posX_mm     = 0.0;
float posY_mm     = 0.0;
float heading_rad = 0.0;

long lastLeftTicksOdo  = 0;
long lastRightTicksOdo = 0;

// Distance-goal control
bool  haveDistanceGoal     = false;
float distanceGoal_mm      = 0.0;  // absolute goal distance
float distanceTravelled_mm = 0.0;  // accumulated |dCenter|
int   distanceDir          = 0;    // 1 = forward, -1 = backward

// Serial input
String inputLine = "";

// ========== Prototypes ==========
void handleSerial();
void updateRPMControl();
void updateOdometry();
void maybeStopAtDistance();
void printStatus();
void resetAll();

void setMotorSpeedLeft(int speed);
void setMotorSpeedRight(int speed);
void applyMotorOutputs();

void leftEncoderISR();
void rightEncoderISR();

// ========== Setup ==========
void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(LEFT_PWM,  OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // Encoder pins
  pinMode(ENC_LEFT_A,   INPUT_PULLUP);
  pinMode(ENC_LEFT_B,   INPUT_PULLUP);
  pinMode(ENC_RIGHT_A,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  resetAll();

  Serial.println("RPM PID + Odometry + Distance serial control");
  Serial.println("Commands:");
  Serial.println("  R<num>  - set target RPM, e.g. R15");
  Serial.println("  F       - forward indefinitely at target RPM");
  Serial.println("  B       - backward indefinitely at target RPM");
  Serial.println("  G<num>  - drive <num> mm (G500, G-300)");
  Serial.println("  S       - stop + cancel distance goal");
  Serial.println("  Z       - reset encoders + odometry");
}

// ========== Main loop ==========
void loop() {
  handleSerial();
  updateOdometry();
  updateRPMControl();
  maybeStopAtDistance();

  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printStatus();
  }
}

// ========== Serial handling ==========
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        inputLine.trim();
        if (inputLine.length() > 0) {
          char cmd = toupper(inputLine.charAt(0));

          if (cmd == 'R') {
            String val = inputLine.substring(1);
            val.trim();
            if (val.length() > 0) {
              float rpm = val.toFloat();
              if (rpm > 0) {
                targetRPM_L = rpm;
                targetRPM_R = rpm;
                Serial.print("Target RPM set to ");
                Serial.println(rpm);
              } else {
                Serial.println("RPM must be > 0");
              }
            } else {
              Serial.println("Use R<num>, e.g. R20");
            }

          } else if (cmd == 'F') {
            haveDistanceGoal     = false;
            distanceTravelled_mm = 0;
            distanceDir          = 0;
            moveDir              = 1;
            pwmLeft  = MIN_PWM_MOVING;
            pwmRight = MIN_PWM_MOVING;
            Serial.print("Forward at ");
            Serial.print(targetRPM_L);
            Serial.println(" RPM");

          } else if (cmd == 'B') {
            haveDistanceGoal     = false;
            distanceTravelled_mm = 0;
            distanceDir          = 0;
            moveDir              = -1;
            pwmLeft  = MIN_PWM_MOVING;
            pwmRight = MIN_PWM_MOVING;
            Serial.print("Backward at ");
            Serial.print(targetRPM_L);
            Serial.println(" RPM");

          } else if (cmd == 'G') {
            String val = inputLine.substring(1);
            val.trim();
            if (val.length() == 0) {
              Serial.println("Use G<num_mm>, e.g. G500 or G-300");
            } else {
              float dist = val.toFloat();
              if (dist == 0) {
                Serial.println("Distance must be non-zero");
              } else {
                haveDistanceGoal     = true;
                distanceGoal_mm      = fabs(dist);
                distanceTravelled_mm = 0.0;
                distanceDir          = (dist > 0) ? 1 : -1;
                moveDir              = distanceDir;
                pwmLeft  = MIN_PWM_MOVING;
                pwmRight = MIN_PWM_MOVING;

                Serial.print("Drive ");
                Serial.print((distanceDir > 0) ? "forward " : "backward ");
                Serial.print(distanceGoal_mm);
                Serial.println(" mm");
              }
            }

          } else if (cmd == 'S') {
            haveDistanceGoal     = false;
            distanceTravelled_mm = 0;
            distanceDir          = 0;
            moveDir              = 0;
            pwmLeft  = 0;
            pwmRight = 0;
            applyMotorOutputs();
            Serial.println("Stop + distance goal cancelled");

          } else if (cmd == 'Z') {
            resetAll();
            Serial.println("Encoders + odometry + PID reset");

          } else {
            Serial.print("Unknown command: ");
            Serial.println(inputLine);
          }
        }
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }
}

// ========== Motor helpers ==========
void setMotorSpeedLeft(int speed) {
  speed = constrain(speed, 0, MAX_PWM);
  analogWrite(LEFT_PWM, speed);
}

void setMotorSpeedRight(int speed) {
  speed = constrain(speed, 0, MAX_PWM);
  analogWrite(RIGHT_PWM, speed);
}

void applyMotorOutputs() {
  if (moveDir == 0) {
    digitalWrite(LEFT_IN1,  LOW);
    digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    setMotorSpeedLeft(0);
    setMotorSpeedRight(0);
    return;
  }

  if (moveDir == 1) {
    // Forward
    digitalWrite(LEFT_IN1,  HIGH);
    digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
  } else {
    // Backward
    digitalWrite(LEFT_IN1,  LOW);
    digitalWrite(LEFT_IN2,  HIGH);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
  }

  setMotorSpeedLeft(pwmLeft);
  setMotorSpeedRight(pwmRight);
}

// ========== Encoder ISRs ==========
void leftEncoderISR() {
  bool a = digitalRead(ENC_LEFT_A);
  bool b = digitalRead(ENC_LEFT_B);
  if (a == b) encLeftCount++;
  else        encLeftCount--;
}

void rightEncoderISR() {
  bool a = digitalRead(ENC_RIGHT_A);
  bool b = digitalRead(ENC_RIGHT_B);
  if (a == b) encRightCount++;
  else        encRightCount--;
}

// ========== RPM-based PID control (your tuning) ==========
void updateRPMControl() {
  unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) return;
  float dt = (now - lastControlTime) / 1000.0f;
  lastControlTime = now;

  long L, R;
  noInterrupts();
  L = encLeftCount;
  R = encRightCount;
  interrupts();

  long dL = L - prevLeftTicks;
  long dR = R - prevRightTicks;

  prevLeftTicks  = L;
  prevRightTicks = R;

  // ticks → RPM
  float factor = 60.0f / (TICKS_PER_REV * dt);
  float rpmL   = fabs(dL * factor);
  float rpmR   = fabs(dR * factor);

  measRPM_L = rpmL;
  measRPM_R = rpmR;

  if (moveDir == 0) {
    pwmLeft = pwmRight = 0;
    applyMotorOutputs();
    return;
  }

  // ----- Left wheel PID (P-only) -----
  float eL   = targetRPM_L - rpmL;
  float outL = Kp_L * eL;
  pwmLeft += (int)outL;
  if (pwmLeft < MIN_PWM_MOVING) pwmLeft = MIN_PWM_MOVING;
  pwmLeft = constrain(pwmLeft, 0, MAX_PWM);

  // ----- Right wheel PID (P-only) -----
  float eR   = targetRPM_R - rpmR;
  float outR = Kp_R * eR;
  pwmRight += (int)outR;
  if (pwmRight < MIN_PWM_MOVING) pwmRight = MIN_PWM_MOVING;
  pwmRight = constrain(pwmRight, 0, MAX_PWM);

  applyMotorOutputs();
}

// ========== Odometry update ==========
void updateOdometry() {
  long leftTicks, rightTicks;
  noInterrupts();
  leftTicks  = encLeftCount;
  rightTicks = encRightCount;
  interrupts();

  long dLeftTicks  = leftTicks  - lastLeftTicksOdo;
  long dRightTicks = rightTicks - lastRightTicksOdo;

  if (dLeftTicks == 0 && dRightTicks == 0) return;

  lastLeftTicksOdo  = leftTicks;
  lastRightTicksOdo = rightTicks;

  float dLeft_mm  = dLeftTicks  * DIST_PER_TICK_MM;
  float dRight_mm = dRightTicks * DIST_PER_TICK_MM;
  float dCenter   = 0.5f * (dLeft_mm + dRight_mm);
  float dTheta    = (dRight_mm - dLeft_mm) / WHEEL_BASE_MM;

  float thetaMid = heading_rad + 0.5f * dTheta;

  posX_mm     += dCenter * cos(thetaMid);
  posY_mm     += dCenter * sin(thetaMid);
  heading_rad += dTheta;

  // Normalize heading to [-pi, pi]
  if (heading_rad > PI)      heading_rad -= 2.0f * PI;
  else if (heading_rad < -PI) heading_rad += 2.0f * PI;

  // Accumulate path length for distance goals
  if (haveDistanceGoal && moveDir != 0) {
    distanceTravelled_mm += fabs(dCenter);
  }
}

// ========== Distance-goal stopping ==========
void maybeStopAtDistance() {
  if (!haveDistanceGoal) return;

  if (distanceTravelled_mm >= distanceGoal_mm) {
    moveDir              = 0;
    pwmLeft              = 0;
    pwmRight             = 0;
    haveDistanceGoal     = false;
    distanceDir          = 0;

    applyMotorOutputs();

    Serial.print("Distance goal reached. Travelled ≈ ");
    Serial.print(distanceTravelled_mm, 1);
    Serial.print(" mm (goal ");
    Serial.print(distanceGoal_mm, 1);
    Serial.println(" mm)");
  }
}

// ========== Reset everything ==========
void resetAll() {
  noInterrupts();
  encLeftCount  = 0;
  encRightCount = 0;
  interrupts();

  prevLeftTicks  = 0;
  prevRightTicks = 0;

  measRPM_L = measRPM_R = 0.0;

  posX_mm     = 0.0;
  posY_mm     = 0.0;
  heading_rad = 0.0;

  lastLeftTicksOdo  = 0;
  lastRightTicksOdo = 0;

  moveDir  = 0;
  pwmLeft  = 0;
  pwmRight = 0;

  integral_L = integral_R = 0.0;
  prevError_L = prevError_R = 0.0;

  haveDistanceGoal     = false;
  distanceGoal_mm      = 0.0;
  distanceTravelled_mm = 0.0;
  distanceDir          = 0;

  applyMotorOutputs();
}

// ========== Status print ==========
void printStatus() {
  long L, R;
  noInterrupts();
  L = encLeftCount;
  R = encRightCount;
  interrupts();

  float mmpsL = (measRPM_L * TICKS_PER_REV / 60.0f) * DIST_PER_TICK_MM;
  float mmpsR = (measRPM_R * TICKS_PER_REV / 60.0f) * DIST_PER_TICK_MM;
  float heading_deg = heading_rad * 180.0f / PI;

  Serial.print("Ticks L: ");
  Serial.print(L);
  Serial.print("  R: ");
  Serial.println(R);

  Serial.print("RPM   L: ");
  Serial.print(measRPM_L, 2);
  Serial.print("  R: ");
  Serial.println(measRPM_R, 2);

  Serial.print("PWM   L: ");
  Serial.print(pwmLeft);
  Serial.print("  R: ");
  Serial.println(pwmRight);

  Serial.print("Speed L: ");
  Serial.print(mmpsL, 1);
  Serial.print(" mm/s   R: ");
  Serial.print(mmpsR, 1);
  Serial.println(" mm/s");

  Serial.print("Pose  x=");
  Serial.print(posX_mm, 1);
  Serial.print(" mm  y=");
  Serial.print(posY_mm, 1);
  Serial.print(" mm  heading=");
  Serial.print(heading_deg, 1);
  Serial.println(" deg");

  Serial.print("Distance travelled for goal: ");
  Serial.print(distanceTravelled_mm, 1);
  Serial.print(" / ");
  Serial.print(distanceGoal_mm, 1);
  Serial.println(" mm");

  Serial.println();
}
