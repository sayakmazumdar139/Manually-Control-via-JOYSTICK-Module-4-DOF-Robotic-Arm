
/*
  4-DOF Robotic Arm with 2x HW-504 Joysticks + Arduino UNO + PCA9685
  - Base, Shoulder, Elbow, Gripper on PCA9685 channels 0..3
  - Smooth analog control with deadzone and per-joint limits
  - Optional: joystick buttons on D2, D3 (not required)

  Hardware notes:
  - Power servos from a separate 5–6V supply to PCA9685 V+; common GND with Arduino.
  - UNO I2C: SDA=A4, SCL=A5
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);  // default PCA9685 address

// ---------- USER SETTINGS ----------
#define PWM_FREQ_HZ        50         // typical for analog servos
#define SERVO_MIN_US       500        // microseconds at 0°
#define SERVO_MAX_US       2500       // microseconds at 180°
#define DEADZONE           40         // joystick dead-zone around center (0..1023 scale)
#define SMOOTH_ALPHA       0.20f      // low-pass smoothing [0..1], higher = snappier
#define LOOP_MS            20         // control loop period (ms)

// PCA9685 channels for each joint:
#define CH_BASE            0
#define CH_SHOULDER        1
#define CH_ELBOW           2
#define CH_GRIPPER         3

// Analog pins for joystick axes:
#define J1_X               A0   // Base
#define J1_Y               A1   // Shoulder
#define J2_X               A2   // Gripper
#define J2_Y               A3   // Elbow

// Optional joystick pushbuttons (active LOW on HW-504)
#define J1_SW              2
#define J2_SW              3

// Per-joint angle limits (tune to your mechanics so nothing binds)
int MIN_ANGLE[4] = {
  10,   // Base
  15,   // Shoulder
  10,   // Elbow
  20    // Gripper (closed)
};
int MAX_ANGLE[4] = {
  170,  // Base
  160,  // Shoulder
  170,  // Elbow
  120   // Gripper (open)
};

// Invert any joint if your servo orientation is reversed
bool INVERT[4] = {
  false,   // Base
  true,    // Shoulder
  false,   // Elbow
  true     // Gripper
};
// ----------------------------------

// Internal state
float currentAngle[4] = {90, 90, 90, 60};  // start at safe-ish mid positions
float targetAngle[4]  = {90, 90, 90, 60};

// Helper: convert microseconds to PCA9685 ticks
uint16_t usToTicks(int us) {
  // PCA9685: 4096 ticks per period
  // Period (us) = 1e6 / freq
  float period_us = 1000000.0f / PWM_FREQ_HZ;
  float ticks = (us * 4096.0f) / period_us;
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

// Helper: write an angle (0..180) to a PCA9685 channel
void writeServoAngle(uint8_t ch, float angleDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  int us = map((int)angleDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint16_t ticks = usToTicks(us);
  pwm.setPWM(ch, 0, ticks);
}

// Normalize a joystick axis (0..1023) to an angle using per-joint limits and deadzone
float axisToAngle(int raw, int jointIndex) {
  // Dead-zone around center (≈512)
  int centered = raw - 512;
  if (abs(centered) < DEADZONE) {
    // Hold current target near center for steadiness
    return targetAngle[jointIndex];
  }

  // Map full range 0..1023 → MIN..MAX
  long mapped = map(raw, 0, 1023, MIN_ANGLE[jointIndex], MAX_ANGLE[jointIndex]);
  float out = (float)mapped;

  // Optional invert
  if (INVERT[jointIndex]) {
    // invert within joint limits
    out = MIN_ANGLE[jointIndex] + (MAX_ANGLE[jointIndex] - (out - MIN_ANGLE[jointIndex]));
  }

  // Constrain
  if (out < MIN_ANGLE[jointIndex]) out = MIN_ANGLE[jointIndex];
  if (out > MAX_ANGLE[jointIndex]) out = MAX_ANGLE[jointIndex];
  return out;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Optional: buttons with internal pullups
  pinMode(J1_SW, INPUT_PULLUP);
  pinMode(J2_SW, INPUT_PULLUP);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);  // 50 Hz for standard servos
  delay(10);

  // Move to initial positions slowly
  for (int i = 0; i < 4; i++) {
    writeServoAngle(i == 0 ? CH_BASE :
                    i == 1 ? CH_SHOULDER :
                    i == 2 ? CH_ELBOW : CH_GRIPPER, currentAngle[i]);
  }

  Serial.println(F("4-DOF Arm ready. Use joysticks to move joints."));
}

unsigned long lastLoop = 0;

void loop() {
  if (millis() - lastLoop < LOOP_MS) return;
  lastLoop = millis();

  // Read joysticks
  int j1x = analogRead(J1_X);  // Base
  int j1y = analogRead(J1_Y);  // Shoulder
  int j2y = analogRead(J2_Y);  // Elbow
  int j2x = analogRead(J2_X);  // Gripper

  // Compute new targets
  targetAngle[0] = axisToAngle(j1x, 0);  // Base
  targetAngle[1] = axisToAngle(j1y, 1);  // Shoulder
  targetAngle[2] = axisToAngle(j2y, 2);  // Elbow
  targetAngle[3] = axisToAngle(j2x, 3);  // Gripper

  // Optional button examples (active LOW):
  // Hold J1_SW to return Base+Shoulder to "home" (90° and 90°)
  if (digitalRead(J1_SW) == LOW) {
    targetAngle[0] = 90;
    targetAngle[1] = 90;
  }
  // Tap J2_SW to snap gripper closed (min); hold to open (max)
  static bool prevJ2 = HIGH;
  bool nowJ2 = digitalRead(J2_SW);
  if (prevJ2 == HIGH && nowJ2 == LOW) {
    // falling edge (tap): close
    targetAngle[3] = MIN_ANGLE[3];
  }
  if (nowJ2 == LOW) {
    // hold: open
    targetAngle[3] = MAX_ANGLE[3];
  }
  prevJ2 = nowJ2;

  // Smooth movement
  for (int i = 0; i < 4; i++) {
    currentAngle[i] = currentAngle[i] + SMOOTH_ALPHA * (targetAngle[i] - currentAngle[i]);
  }

  // Write to servos
  writeServoAngle(CH_BASE,     currentAngle[0]);
  writeServoAngle(CH_SHOULDER, currentAngle[1]);
  writeServoAngle(CH_ELBOW,    currentAngle[2]);
  writeServoAngle(CH_GRIPPER,  currentAngle[3]);

  // Debug (optional): prints at ~50Hz/LOOP_MS
  // Serial.print("B:"); Serial.print((int)currentAngle[0]);
  // Serial.print(" S:"); Serial.print((int)currentAngle[1]);
  // Serial.print(" E:"); Serial.print((int)currentAngle[2]);
  // Serial.print(" G:"); Serial.println((int)currentAngle[3]);
}