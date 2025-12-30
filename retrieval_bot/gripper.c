#include <Arduino.h>
#include <PS4Controller.h>
#include <ESP32Servo.h>

#define SERVO_PIN 18
#define MIN_ANGLE 5            // safer mechanical limits (adjust if tested)
#define MAX_ANGLE 175
#define TRIGGER_MAX_DEG_PER_S 100   // raise/lower to taste
#define DEADZONE 12                 // ignore tiny trigger noise (0..255)

// State
Servo gripper;
float angleDeg = (MIN_ANGLE + MAX_ANGLE) * 0.5f;
uint32_t lastMs = 0;

void setup() {
  Serial.begin(115200);
  PS4.begin();
  Serial.println("Waiting for PS4 controller... (Hold SHARE + PS)");

  // ESP32Servo best-practice setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gripper.setPeriodHertz(50);                 // 50 Hz for servos
  gripper.attach(SERVO_PIN, 500, 2500);       // widen if your servo supports it

  gripper.write((int)round(angleDeg));
  lastMs = millis();
}

void loop() {
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  // Cap dt to avoid big jumps after stalls
  if (dt > 0.05f) dt = 0.05f; // treat anything >50 ms as 50 ms

  if (!PS4.isConnected()) {
    // Optional: move to a known-safe angle here if you prefer
    delay(10);
    return;
  }

  // Instant open/close
  if (PS4.R1()) angleDeg = MAX_ANGLE;
  if (PS4.L1()) angleDeg = MIN_ANGLE;

  // Proportional trigger control with deadzone
  int r2 = PS4.R2Value(); // open
  int l2 = PS4.L2Value(); // close

  if (r2 > DEADZONE || l2 > DEADZONE) {
    // map() is int-based; casting keeps it clean
    float openSpd  = map(r2, DEADZONE, 255, 0, TRIGGER_MAX_DEG_PER_S);
    float closeSpd = map(l2, DEADZONE, 255, 0, TRIGGER_MAX_DEG_PER_S);
    if (openSpd   < 0) openSpd = 0;   // when r2==DEADZONE, map can go slightly negative
    if (closeSpd  < 0) closeSpd = 0;

    float net = openSpd - closeSpd;   // + = open, - = close
    angleDeg = constrain(angleDeg + net * dt, MIN_ANGLE, MAX_ANGLE);
  }

  // Only send a pulse if we actually changed by 0.5°
  static float lastCmd = -999;
  if (fabs(angleDeg - lastCmd) >= 0.5f) {
    gripper.write((int)round(angleDeg));
    lastCmd = angleDeg;
  }
  delay(2);
}
