#include <math.h>
#include <Servo.h>   // <<< ADDED

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

// ========== Servo ==========
const int SERVO_PIN = 34;    // <<< your servo signal pin
Servo myServo;

// ========== Encoder / wheel constants ==========
const float TICKS_PER_REV      = 1920.0;
const float WHEEL_DIAMETER_MM  = 62.0;
const float WHEEL_CIRCUM_MM    = PI * WHEEL_DIAMETER_MM;
const float DIST_PER_TICK_MM   = WHEEL_CIRCUM_MM / TICKS_PER_REV;

// ========== Control constants ==========
const int   MIN_PWM_MOVING     = 20;     
const int   MAX_PWM            = 255;

// ========== Encoder state ==========
volatile long encLeftCount  = 0;
volatile long encRightCount = 0;

long prevLeftTicks  = 0;
long prevRightTicks = 0;

float measRPM_L = 0.0;
float measRPM_R = 0.0;

// ========== Motion / PID state ==========
//  1 = forward, -1 = backward, 0 = stop
int moveDir = 1;  // auto-start forward

// Starting target RPM (same both wheels)
float targetRPM_L = 15.0;
float targetRPM_R = 15.0;

// PID gains
float Kp_L = 1.0, Ki_L = 0.01, Kd_L = 0.01;
float Kp_R = 1.0, Ki_R = 0.01, Kd_R = 0.001;

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

// ========== Prototypes ==========
void updateRPMControl();
void printStatus();
void setMotorSpeedLeft(int speed);
void setMotorSpeedRight(int speed);
void applyMotorOutputs();
void resetEncodersAndControl();
void leftEncoderISR();
void rightEncoderISR();
void handleServoSerial();   // <<< ADDED

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

  // Servo setup
  myServo.attach(SERVO_PIN);   // <<< servo on pin 34
  myServo.write(90);           // center position to start

  resetEncodersAndControl();

  Serial.println("System booting...");
  Serial.println("Servo on pin 34. Send angle 0-180 to move it.");
  Serial.println("Waiting 5 seconds before starting motion...");
  delay(5000);

  Serial.print("Starting forward at ");
  Serial.print(targetRPM_L);
  Serial.println(" RPM");

  moveDir = 1;                    // forward
  pwmLeft  = MIN_PWM_MOVING;      // give both motors some initial PWM
  pwmRight = MIN_PWM_MOVING;
}

// ========== Main loop ==========
void loop() {
  updateRPMControl();

  unsigned long now = millis();
  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printStatus();
  }

  // Check for servo commands over Serial
  handleServoSerial();   // <<< ADDED
}

// ========== Servo Serial handler ==========
void handleServoSerial() {
  // Simple protocol:
  //   type an integer 0–180 and press Enter -> servo moves to that angle
  if (Serial.available() > 0) {
    int angle = Serial.parseInt();     // reads next integer from serial
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
      Serial.print("Servo -> ");
      Serial.print(angle);
      Serial.println(" degrees");
    } else {
      // clear non-numeric junk
      while (Serial.available() > 0) Serial.read();
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
    // Stop
    digitalWrite(LEFT_IN1,  LOW);
    digitalWrite(LEFT_IN2,  LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    setMotorSpeedLeft(0);
    setMotorSpeedRight(0);
    return;
  }

  if (moveDir == 1) {
    // Forward (both wheels forward)
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

  // ----- Left wheel PID (P-only here) -----
  float eL   = targetRPM_L - rpmL;
  float outL = Kp_L * eL;
  pwmLeft += (int)outL;
  if (pwmLeft < MIN_PWM_MOVING) pwmLeft = MIN_PWM_MOVING;
  pwmLeft = constrain(pwmLeft, 0, MAX_PWM);

  // ----- Right wheel PID -----
  float eR   = targetRPM_R - rpmR;
  float outR = Kp_R * eR;
  pwmRight += (int)outR;
  if (pwmRight < MIN_PWM_MOVING) pwmRight = MIN_PWM_MOVING;
  pwmRight = constrain(pwmRight, 0, MAX_PWM);

  applyMotorOutputs();
}

// ========== Helpers ==========
void resetEncodersAndControl() {
  noInterrupts();
  encLeftCount  = 0;
  encRightCount = 0;
  interrupts();

  prevLeftTicks  = 0;
  prevRightTicks = 0;

  measRPM_L = measRPM_R = 0.0;

  moveDir = 0;
  pwmLeft = pwmRight = 0;

  integral_L = integral_R = 0.0;
  prevError_L = prevError_R = 0.0;
}

void printStatus() {
  long L, R;
  noInterrupts();
  L = encLeftCount;
  R = encRightCount;
  interrupts();

  float mmpsL = (measRPM_L * TICKS_PER_REV / 60.0f) * DIST_PER_TICK_MM;
  float mmpsR = (measRPM_R * TICKS_PER_REV / 60.0f) * DIST_PER_TICK_MM;

  Serial.print("Ticks L: ");
  Serial.print(L);
  Serial.print("  R: ");
  Serial.println(R);

  Serial.print("RPM L: ");
  Serial.print(measRPM_L, 2);
  Serial.print("  R: ");
  Serial.println(measRPM_R, 2);

  Serial.print("PWM L: ");
  Serial.print(pwmLeft);
  Serial.print("  PWM R: ");
  Serial.println(pwmRight);

  Serial.print("Speed L: ");
  Serial.print(mmpsL, 1);
  Serial.print(" mm/s   R: ");
  Serial.print(mmpsR, 1);
  Serial.println(" mm/s");

  Serial.println();
}
