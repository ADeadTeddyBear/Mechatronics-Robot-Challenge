#include <Servo.h>
#include <math.h>

// ===== Servo pin assignments =====
const int SERVO1_PIN = 34;
const int SERVO2_PIN = 35;
const int SERVO3_PIN = 37;

Servo servo1;
Servo servo2;
Servo servo3;

// Servo angles
int angle1 = 90;
int angle2 = 140;
int angle3 = 0;

// ===== Ultrasonic sensor pins (change to match your wiring) =====
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

// ===== Auto routine state machine =====
enum AutoState {
  AUTO_WAIT_5S,
  AUTO_DRIVE_TO_25,
  AUTO_TURN_90,
  AUTO_DONE
};

AutoState autoState = AUTO_WAIT_5S;
unsigned long autoStartTime = 0;

// == Turn control using odometry ==
const float TURN_ANGLE_RAD = PI / 2.8;   // target ~90°
float turnStartHeading = 0.0;

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

// PID gains (as in your script)
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

// Distance-goal control (kept for G<num> serial command)
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

void updateAutoRoutine(unsigned long now);
float getDistanceCm();
float angleDiff(float a, float b);  // NEW helper

// ========== Setup ==========
void setup() {
  Serial.begin(9600);

  // ----- Servo setup -----
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  angle1 = 90;
  angle2 = 140;
  angle3 = 0;

  servo1.write(angle1);  // S1 = 90
  servo2.write(angle2);  // S2 = 140
  servo3.write(angle3);  // S3 = 0

  autoStartTime = millis();
  autoState     = AUTO_WAIT_5S;

  // ----- Ultrasonic pins -----
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ----- Motor pins -----
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(LEFT_PWM,  OUTPUT);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // ----- Encoder pins -----
  pinMode(ENC_LEFT_A,   INPUT_PULLUP);
  pinMode(ENC_LEFT_B,   INPUT_PULLUP);
  pinMode(ENC_RIGHT_A,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_B,  INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

  resetAll();

  Serial.println("Servo + PID Motors + Sensor-based 25cm stop + 90deg turn via ODOMETRY");
  Serial.println("Startup auto sequence:");
  Serial.println("  S1=90, S2=140, S3=0");
  Serial.println("  wait 5s, S3->60");
  Serial.println("  drive forward until distance <= 25 cm, then turn ~90° right using odometry and stop.");
  Serial.println();
  Serial.println("Motor serial commands still available:");
  Serial.println("  R<num>  - set target RPM, e.g. R15");
  Serial.println("  F       - forward indefinitely at target RPM");
  Serial.println("  B       - backward indefinitely at target RPM");
  Serial.println("  G<num>  - drive <num> mm using odometry (G500, G-300)");
  Serial.println("  S       - stop + cancel distance goal");
  Serial.println("  Z       - reset encoders + odometry");
}

// ========== Main loop ==========
void loop() {
  unsigned long now = millis();

  handleSerial();
  updateOdometry();
  updateRPMControl();
  maybeStopAtDistance();   // for G<num> commands
  updateAutoRoutine(now);  // for the startup 25cm + turn behaviour

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printStatus();
  }
}

// ========== Auto routine (servo start + 25cm via sensor + 90° turn) ==========
void updateAutoRoutine(unsigned long now) {
  switch (autoState) {

    case AUTO_WAIT_5S:
      // Wait 5 seconds after startup
      if (now - autoStartTime >= 5000UL) {
        angle3 = 60;
        servo3.write(angle3);
        Serial.println("Auto: Servo 3 -> 60 deg");

        // Start driving forward under PID control (no distance goal)
        haveDistanceGoal     = false;
        distanceTravelled_mm = 0.0;
        distanceDir          = 0;
        moveDir              = 1;                // forward
        pwmLeft              = MIN_PWM_MOVING;
        pwmRight             = MIN_PWM_MOVING;
        Serial.println("Auto: driving forward using PID until distance <= 25 cm");

        autoState = AUTO_DRIVE_TO_25;
      }
      break;

    case AUTO_DRIVE_TO_25: {
      float dist = getDistanceCm();
      if (dist > 0) {   // valid reading
        Serial.print("Auto distance: ");
        Serial.print(dist);
        Serial.println(" cm");

        if (dist <= 15.0) {
          // Stop the PID-driven forward motion
          moveDir  = 0;
          pwmLeft  = 0;
          pwmRight = 0;
          applyMotorOutputs();
          Serial.println("Auto: reached 20 cm from wall, stopping and starting odometry-based turn...");

          // === Start in-place right turn (left fwd, right back) with fixed LOW PWM ===
          int turnPWM = 70;  // LOWER SPEED to avoid overshoot – you can tune this
          digitalWrite(LEFT_IN1,  HIGH);
          digitalWrite(LEFT_IN2,  LOW);
          digitalWrite(RIGHT_IN1, LOW);
          digitalWrite(RIGHT_IN2, HIGH);
          analogWrite(LEFT_PWM,  turnPWM);
          analogWrite(RIGHT_PWM, turnPWM);

          // Save heading at start of turn
          turnStartHeading = heading_rad;
          Serial.print("Auto: turn start heading (deg) = ");
          Serial.println(turnStartHeading * 180.0 / PI);

          autoState = AUTO_TURN_90;
        }
      }
      break;
    }

    case AUTO_TURN_90: {
      // Use odometry-based heading to decide when we've turned ~90°
      float dHeading = angleDiff(heading_rad, turnStartHeading); // signed shortest difference
      float dDeg = dHeading * 180.0 / PI;

      Serial.print("Auto turning: dHeading = ");
      Serial.print(dDeg);
      Serial.println(" deg");

      // For a right turn, dHeading will be negative. Use fabs for generality.
      if (fabs(dHeading) >= TURN_ANGLE_RAD) {
        // Stop motors
        digitalWrite(LEFT_IN1,  LOW);
        digitalWrite(LEFT_IN2,  LOW);
        digitalWrite(RIGHT_IN1, LOW);
        digitalWrite(RIGHT_IN2, LOW);
        analogWrite(LEFT_PWM,  0);
        analogWrite(RIGHT_PWM, 0);

        moveDir  = 0;
        pwmLeft  = 0;
        pwmRight = 0;

        autoState = AUTO_DONE;
        Serial.print("Auto: turn complete using odometry. Total turn ≈ ");
        Serial.print(dDeg);
        Serial.println(" deg, robot stopped.");
      }
      break;
    }

    case AUTO_DONE:
    default:
      // Do nothing; user can control with serial commands from here
      break;
  }
}

// ========== Ultrasonic distance function ==========
float getDistanceCm() {
  // Trigger a 10 µs pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo (timeout ~30 ms)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000L); // microseconds

  if (duration == 0) {
    // no echo
    return -1.0;
  }

  // Speed of sound ~343 m/s -> 0.0343 cm/us
  float distance = (duration * 0.0343f) / 2.0f;
  return distance;
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

// ========== RPM-based PID control ==========
void updateRPMControl() {
  unsigned long now = millis();

  // During the odometry-based turn, don't let PID fight our manual spin
  if (autoState == AUTO_TURN_90) {
    lastControlTime = now;  // keep dt small for when we resume
    return;
  }

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
  if (heading_rad > PI)       heading_rad -= 2.0f * PI;
  else if (heading_rad < -PI) heading_rad += 2.0f * PI;

  // Accumulate path length for distance goals
  if (haveDistanceGoal && moveDir != 0) {
    distanceTravelled_mm += fabs(dCenter);
  }
}

// ========== Distance-goal stopping (for G<num>) ==========
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

// ========== Angle difference helper (wrap-safe) ==========
float angleDiff(float a, float b) {
  float d = a - b;
  while (d > PI)  d -= 2.0f * PI;
  while (d < -PI) d += 2.0f * PI;
  return d;
}
