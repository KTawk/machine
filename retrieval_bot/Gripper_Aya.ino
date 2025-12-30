#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18  // change if needed

Servo servo;
int servoAngle ;   // start centered
int lastR2 = 0;
int lastL2 = 0;

void setup() {
  Serial.begin(115200);
  PS4.begin();
  Serial.println("Waiting for PS4 controller... (Hold SHARE + PS)");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2500);
  //servo.write(servoAngle);
}

void loop() {
  if (!PS4.isConnected()) return;

  int r2 = PS4.R2Value();
  int l2 = PS4.L2Value();

  // --- R2 increases: move → 180° (OPEN FROM A GIVEN ANGLE) ---
  if (r2 > lastR2 && r2 > 20) {
    servo.write(180);
    Serial.print("R2 = "); Serial.print(r2);
    //Serial.print("  → Servo angle = "); Serial.println(servoAngle);
  }

  // --- L2 increases: move 180° → 0° (CLOSE)---
  else if (l2 > lastL2 && l2 > 20) {
    servoAngle = map(l2, 20, 255, 180, 0);
    servo.write(servoAngle);
    Serial.print("L2 = "); Serial.print(l2);
    Serial.print("  → Servo angle = "); Serial.println(servoAngle);
  }

  // Remember last trigger states
  lastR2 = r2;
  lastL2 = l2;

  delay(20);
}

