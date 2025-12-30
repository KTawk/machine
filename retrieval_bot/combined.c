#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>

#define SERVOR_PIN    18
#define SERVOL_PIN    21
#define GRIPPER_PIN   19

#define NEUTRAL_US   1530   // your calibrated stop pulse --> servo deadzone
#define MAX_US       2000   // "forward" (increase speed)
#define MIN_US       1000   // "reverse" (decrease speed)
#define DEADZONE     15     // |stick| <= DEADZONE => neutral. --> stick deadzone
#define INVERT_STICK true   // PS4 Y is negative when UP; true makes UP positive

Servo servoR;
Servo servoL;
Servo gripper;
int gripperAngle;
int gripperLastR2 = 0;
int gripperLastL2 = 0;

static inline bool isNeutral(int v) { return abs(v) <= DEADZONE; }

void setup() {
  Serial.begin(115200);
  Serial.println("Waiting for PS4 controller... (Hold SHARE + PS)");

  PS4.begin();

  // ESP32Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoR.setPeriodHertz(50); // 50 Hz for hobby servos
  servoL.setPeriodHertz(50);
  gripper.setPeriodHertz(50);

  // Attach with safe pulse range limits
  servoR.attach(SERVOR_PIN, 1000, 2000);
  servoL.attach(SERVOL_PIN, 1000, 2000);
  gripper.attach(GRIPPER_PIN, 500, 2500);

  servoR.writeMicroseconds(NEUTRAL_US); // start stopped
  servoL.writeMicroseconds(NEUTRAL_US);

  Serial.println("Ready: Both sticks UP/DOWN -> move both. Single stick -> that side only (other must be neutral).");
}

void loop() {
  if (!PS4.isConnected()) {
    servoR.writeMicroseconds(NEUTRAL_US);
    servoL.writeMicroseconds(NEUTRAL_US);
    delay(20);
    return;
  }

  // Read sticks Y in [-128..+127]
  int stickL = PS4.LStickY();
  int stickR = PS4.RStickY();
  // For gripper control
  int r2 = PS4.R2Value();
  int l2 = PS4.L2Value();

  // Make UP positive on both sticks if requested
  if (INVERT_STICK) {
    stickL = stickL;
    stickR = stickR;
  }

  const bool L_neutral = isNeutral(stickL);
  const bool R_neutral = isNeutral(stickR);

  int pulseL = -NEUTRAL_US;  // safe defaults
  int pulseR = -NEUTRAL_US;

  // -------- Priority 1: BOTH STICKS (drive together) --------
  if (!L_neutral && !R_neutral) {
    if (stickL > DEADZONE && stickR > DEADZONE) {
      // both UP -> forward
      pulseL = MAX_US;
      pulseR = MIN_US;
    } else if (stickL < -DEADZONE && stickR < -DEADZONE) {
      // both DOWN -> backward
      pulseL = MIN_US;
      pulseR = MAX_US;
    } else if (stickL > DEADZONE && stickR < -DEADZONE) {
      // both UP -> forward
      pulseL = MAX_US;
      pulseR = MAX_US;
    } else if (stickL < -DEADZONE && stickR > DEADZONE) {
      // both DOWN -> backward
      pulseL = MIN_US;
      pulseR = MIN_US;
    }else {
      // conflicting directions (e.g., one up, one down) -> stop (or choose your own behavior)
      pulseL = NEUTRAL_US;
      pulseR = NEUTRAL_US;
    }
  }
  // -------- Priority 2: SINGLE STICK (other must be neutral) --------
  else if (!L_neutral && R_neutral) {
    // left stick only -> left servo
    pulseL = (stickL > 0) ? MAX_US : MIN_US;
    pulseR = NEUTRAL_US;
  }
  else if (!R_neutral && L_neutral) {
    // right stick only -> right servo
    pulseR = (stickR > 0) ? MIN_US : MAX_US;
    pulseL = NEUTRAL_US;
  }
  // -------- Priority 3: both neutral -> stop --------
  else {
    pulseL = NEUTRAL_US;
    pulseR = NEUTRAL_US;
  }

  // Send pulses
  servoR.writeMicroseconds(pulseR);
  servoL.writeMicroseconds(pulseL);

  // Debug print
  Serial.printf("L:%4d (-> %4dus)  |  R:%4d (-> %4dus)\n", stickL, pulseL, stickR, pulseR);





  // --- GRIPPER CONTROL ---
  // --- R2 increases: move → 180° (OPEN FROM A GIVEN ANGLE) ---
  if (r2 > gripperLastR2 && r2 > 20) {
    gripper.write(60);
    Serial.print("R2 = "); Serial.print(r2);
    //Serial.print("  → Servo angle = "); Serial.println(servoAngle);
  }

  // --- L2 increases: move 180° → 0° (CLOSE)---
  else if (l2 > gripperLastL2 && l2 > 20) {
    gripperAngle = map(l2, 20, 255, 60, 0);
    gripper.write(gripperAngle);
    Serial.print("L2 = "); Serial.print(l2);
    Serial.print("  → Servo angle = "); Serial.println(gripperAngle);
  }

  // Remember last trigger states
  gripperLastR2 = r2;
  gripperLastL2 = l2;
  delay(10);
}
