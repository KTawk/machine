#include <Servo.h>

Servo servo;

const int SERVO_PIN = 26;

// Pulse range for 270° servo (adjust if needed)
const int MIN_US = 500;    // 0°
const int MAX_US = 2500;   // 270°

const int STEP_US = 5;
const int STEP_DELAY = 10;

void setup() {
  servo.attach(SERVO_PIN, MIN_US, MAX_US);

  // Start at 270°
  servo.writeMicroseconds(MAX_US);
  delay(1000);
}

void loop() {
  // Sweep from 270° → 0°
  for (int us = MAX_US; us >= MIN_US; us -= STEP_US) {
    servo.writeMicroseconds(us);
    delay(STEP_DELAY);
  }

  // Hold at 0°
  servo.writeMicroseconds(MIN_US);

  // Stop forever
  while (true) {
    delay(1000);
  }
}
