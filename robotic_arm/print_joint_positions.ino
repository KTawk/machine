#include <Arduino.h>
#include <ESP32Servo.h>

// ================= SERVO PINS =================
#define BASE_PIN        27
#define SHOULDER_PIN    14
#define ELBOW_PIN       12
#define WRIST_ROT_PIN   16
#define WRIST_BEND_PIN  13

// ================= SERVO OBJECTS =================
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoWristRot;
Servo servoWristBend;

// ================= ANGLE LIMITS =================
// Adjust if needed
#define MIN_DEG  20
#define MAX_DEG  200
#define STEP_DEG 10

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Attach servos
  servoBase.attach(BASE_PIN);
  servoShoulder.attach(SHOULDER_PIN);
  servoElbow.attach(ELBOW_PIN);
  servoWristRot.attach(WRIST_ROT_PIN);
  servoWristBend.attach(WRIST_BEND_PIN);

  Serial.println("=== SERVO POSITION TEST START ===");
}

void moveAndPrint(int angle) {
  //servoBase.write(angle);
  servoShoulder.write(angle);
  //servoElbow.write(angle);
  //servoWristRot.write(angle);
  //servoWristBend.write(angle);

  Serial.print("Angle sent -> ");
  //Serial.print("BASE: "); Serial.print(angle);
  Serial.print(" | SHOULDER: "); Serial.print(angle);
  //Serial.print(" | ELBOW: "); Serial.print(angle);
  //Serial.print(" | WRIST ROT: "); Serial.print(angle);
  //Serial.print(" | WRIST BEND: "); Serial.println(angle);
}

void loop() {
  // Sweep UP
  for (int angle = MIN_DEG; angle <= MAX_DEG; angle += STEP_DEG) {
    moveAndPrint(angle);
    delay(5000);
  }

  delay(2000);

  // Sweep DOWN
  for (int angle = MAX_DEG; angle >= MIN_DEG; angle -= STEP_DEG) {
    moveAndPrint(angle);
    delay(5000);
  }

  delay(3000);
}
