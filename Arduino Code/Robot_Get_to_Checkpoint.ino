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

// ===== Ultrasonic sensor pins =====
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

// ===== Auto routine state machine =====
enum AutoState {
  AUTO_WAIT_5S,                // wait 5 seconds with servos at home (S3 = 0)
  AUTO_DRIVE_TO_3,             // drive forward until distance <= 3 cm
  AUTO_BACK_TO_20,             // drive backward until distance >= 20 cm
  AUTO_TURN_90,                // 90° right turn using odometry
  AUTO_DRIVE_TO_20_AFTER_TURN, // drive forward until distance <= 20 cm
  AUTO_DONE
};

AutoState autoState = AUTO_WAIT_5S;
unsigned long autoStartTime = 0;

// ======= TURN TUNING SECTION (odometry-based in-place turn) =======
const float TARGET_TURN_DEG     = 85.0;  // desired nominal turn
const float TURN_SCALE          = 0.95;  // <1.0 if it over-turns, >1.0 if under-turns
const float TURN_TOL_DEG        = 2.0;   // how close is "good enough"

const int   TURN_PWM_FAST       = 60;    // PWM at start of turn
const int   TURN_PWM_SLOW       = 30;    // PWM near the end of turn
const float SLOWDOWN_START_DEG  = 30.0;  // when to switch to slower PWM

bool  turning          = false; // true while we're in an in-place turn
float turnStartHeading = 0.0;   // heading at start of current turn

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
const int ENC_LEFT_A   = 21;
const int ENC_LEFT_B   = 20;
const int ENC_RIGHT_A  = 19;
const int ENC_RIGHT_B  = 18;

// ========== Encoder / wheel constants ==========
const float TICKS_PER_REV      = 1920.0;
const float WHEEL_DIAMETER_MM  = 60.0;
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

// PID gains
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

// Distance-goal control (for G<num> serial command)
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
float angleDiff(float a, float b);  // wrap-safe angle difference

// Turn helpers (from perfect steering script)
void startRightTurn();
void stopAllMotors();
void setTurnPWM(int pwm);

// ========== Setup ==========
void setup() {
  Serial.begin(9600);

  // ----- Servo setup -----
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  // Home pose
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

  Serial.println("Servo + PID Motors + Sensor-based sequence using ODOMETRY turn");
  Serial.println("Startup auto sequence:");
  Serial.println("  1) Home servos: S1=90, S2=140, S3=0");
  Serial.println("  2) Wait 5 s, then S3->90");
  Serial.println("  3) Drive FORWARD until distance <= 3 cm");
  Serial.println("  4) Drive BACKWARD until distance >= 20 cm");
  Serial.println("  5) Turn ~90° right using odometry (tuned section at top)");
  Serial.println("  6) Drive FORWARD until distance <= 20 cm, then stop.");
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
  updateAutoRoutine(now);  // for the startup behaviour

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printStatus();
  }
}

// ========== Auto routine ==========
void updateAutoRoutine(unsigned long now) {
  switch (autoState) {

    case AUTO_WAIT_5S:
      // Wait 5 seconds after startup with S3 = 0
      if (now - autoStartTime >= 5000UL) {
        // Open gripper
        angle3 = 90;
        servo3.write(angle3);
        Serial.println("Auto: Servo 3 -> 90 deg");

        // Start driving forward under PID control (no distance goal)
        haveDistanceGoal     = false;
        distanceTravelled_mm = 0.0;
        distanceDir          = 0;
        moveDir              = 1;                // forward
        pwmLeft              = MIN_PWM_MOVING;
        pwmRight             = MIN_PWM_MOVING;
        Serial.println("Auto: driving FORWARD until distance <= 3 cm");

        autoState = AUTO_DRIVE_TO_3;
      }
      break;

    case AUTO_DRIVE_TO_3: {
      float dist = getDistanceCm();
      if (dist > 0) {   // valid reading
        Serial.print("Auto (forward) distance: ");
        Serial.print(dist);
        Serial.println(" cm");

        if (dist <= 3.0) {
          // Stop forward motion
          moveDir  = 0;
          pwmLeft  = 0;
          pwmRight = 0;
          applyMotorOutputs();
          Serial.println("Auto: reached <= 3 cm, stopping and reversing to 20 cm...");

          // Start driving backward under PID control
          haveDistanceGoal     = false;
          distanceTravelled_mm = 0.0;
          distanceDir          = 0;
          moveDir              = -1;               // backward
          pwmLeft              = MIN_PWM_MOVING;
          pwmRight             = MIN_PWM_MOVING;

          autoState = AUTO_BACK_TO_20;
        }
      }
      break;
    }

    case AUTO_BACK_TO_20: {
      float dist = getDistanceCm();
      if (dist > 0) {
        Serial.print("Auto (backward) distance: ");
        Serial.print(dist);
        Serial.println(" cm");

        // Started near 3 cm, backing away until we reach >= 20 cm
        if (dist >= 20.0) {
          // Stop backward motion
          moveDir  = 0;
          pwmLeft  = 0;
          pwmRight = 0;
          applyMotorOutputs();
          Serial.println("Auto: reached >= 20 cm while reversing, starting 90° turn...");

          // === Start in-place right turn using odometry-based tuner ===
          startRightTurn();        // sets direction + PWM + records turnStartHeading
          autoState = AUTO_TURN_90;
        }
      }
      break;
    }

    case AUTO_TURN_90: {
      if (!turning) {
        // Safety: if somehow turning flag is false, do nothing
        break;
      }

      // How far have we turned since we started?
      float dHeading = angleDiff(heading_rad, turnStartHeading); // signed shortest diff
      float turnedDeg = fabs(dHeading) * 180.0 / PI;

      // Choose PWM based on how far we've turned
      int pwm;
      if (turnedDeg < SLOWDOWN_START_DEG) {
        pwm = TURN_PWM_FAST;
      } else {
        pwm = TURN_PWM_SLOW;
      }
      setTurnPWM(pwm);

      // Effective target angle we want (scaled)
      float targetDegEffective = TARGET_TURN_DEG * TURN_SCALE;

      // Debug print (optional; comment out if too chatty)
      Serial.print("Auto turning: turned = ");
      Serial.print(turnedDeg, 1);
      Serial.print(" deg  (target≈ ");
      Serial.print(targetDegEffective, 1);
      Serial.println(" deg)");

      // Stop when we've reached (or slightly passed) the target window
      if (turnedDeg >= targetDegEffective - TURN_TOL_DEG) {
        stopAllMotors();
        turning = false;

        Serial.print("Auto: turn complete using odometry. Total turn ≈ ");
        Serial.print(turnedDeg, 1);
        Serial.println(" deg.");

        // After the turn, drive forward until distance <= 20 cm
        haveDistanceGoal     = false;
        distanceTravelled_mm = 0.0;
        distanceDir          = 0;
        moveDir              = 1;                // forward
        pwmLeft              = MIN_PWM_MOVING;
        pwmRight             = MIN_PWM_MOVING;
        Serial.println("Auto: driving FORWARD until distance <= 20 cm after turn");

        autoState = AUTO_DRIVE_TO_20_AFTER_TURN;
      }
      break;
    }

    case AUTO_DRIVE_TO_20_AFTER_TURN: {
      float dist = getDistanceCm();
      if (dist > 0) {
        Serial.print("Auto (after turn) distance: ");
        Serial.print(dist);
        Serial.println(" cm");

        if (dist <= 20.0) {   // **20 cm** as per your description
          // Stop forward motion
          moveDir  = 0;
          pwmLeft  = 0;
          pwmRight = 0;
          applyMotorOutputs();

          autoState = AUTO_DONE;
          Serial.println("Auto: reached <= 20 cm after turn, sequence complete. Robot stopped.");
        }
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

// ========== Serial handling (unchanged motor commands) ==========
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

  turning          = false;
  turnStartHeading = 0.0;

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

// ========== TURN HELPERS (from perfect steering) ==========
void startRightTurn() {
  // Save where we started
  turnStartHeading = heading_rad;
  turning = true;

  Serial.print("Starting right turn. Start heading = ");
  Serial.print(turnStartHeading * 180.0 / PI, 1);
  Serial.println(" deg");

  // Set direction: left forward, right backward (right turn)
  digitalWrite(LEFT_IN1,  HIGH);
  digitalWrite(LEFT_IN2,  LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, HIGH);

  setTurnPWM(TURN_PWM_FAST);
}

void stopAllMotors() {
  digitalWrite(LEFT_IN1,  LOW);
  digitalWrite(LEFT_IN2,  LOW);
  digitalWrite(RIGHT_IN1, LOW);
  digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_PWM,  0);
  analogWrite(RIGHT_PWM, 0);
}

void setTurnPWM(int pwm) {
  if (!turning) {
    // Safety: don't spin if not in "turning" mode
    analogWrite(LEFT_PWM,  0);
    analogWrite(RIGHT_PWM, 0);
    return;
  }

  pwm = constrain(pwm, 0, 255);
  analogWrite(LEFT_PWM,  pwm);
  analogWrite(RIGHT_PWM, pwm);
}
