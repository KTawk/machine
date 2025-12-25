#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ================= PINS =================
#define BASE_PIN        27
#define SHOULDER_PIN    14
#define ELBOW_PIN       12
#define WRIST_ROT_PIN   16
#define WRIST_BEND_PIN  13

// ================= SERVO LIMITS (µs) =================
#define BASE_MIN_US        700
#define BASE_MAX_US       2300
#define SHOULDER_MIN_US    700
#define SHOULDER_MAX_US   2300
#define ELBOW_MIN_US       700
#define ELBOW_MAX_US      2300
#define WRIST_ROT_MIN_US   700
#define WRIST_ROT_MAX_US  2300
#define WRIST_BEND_MIN_US  700
#define WRIST_BEND_MAX_US 2300

// ================= ANGLE LIMITS (deg) =================
#define BASE_MIN_DEG        10
#define BASE_MAX_DEG       260
#define SHO_MIN_DEG         10
#define SHO_MAX_DEG        260
#define ELB_MIN_DEG         10
#define ELB_MAX_DEG        260
#define WRIST_ROT_MIN_DEG    0
#define WRIST_ROT_MAX_DEG  270
#define WRIST_BEND_MIN_DEG   0
#define WRIST_BEND_MAX_DEG 180

// ================= CONTROL =================
#define DEADZONE   40
#define LOOP_DELAY 20

// ================= SERVOS =================
Servo baseServo, shoulderServo, elbowServo;
Servo wristRotServo, wristBendServo;

// ================= CONTROLLER =================
GamepadPtr gp = nullptr;

// ================= CURRENT ANGLES (SAME AS YOUR CODE) =================
float baseDeg      = 135;
float shoDeg       = 90;
float elbDeg       = 90;
float wristRotDeg  = 135;
float wristBendDeg = 90;

// ================= HELPERS =================
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

int degToUs(float deg, float dMin, float dMax, int usMin, int usMax) {
  deg = clampf(deg, dMin, dMax);
  float t = (deg - dMin) / (dMax - dMin);
  return (int)lroundf(usMin + t * (usMax - usMin));
}

// ================= CALLBACKS =================
void onConnected(GamepadPtr ctl) {
  gp = ctl;
  Serial.println("Controller connected");
}

void onDisconnected(GamepadPtr ctl) {
  if (gp == ctl) gp = nullptr;
  Serial.println("Controller disconnected");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  BP32.setup(&onConnected, &onDisconnected);

  baseServo.attach(BASE_PIN, BASE_MIN_US, BASE_MAX_US);
  shoulderServo.attach(SHOULDER_PIN, SHOULDER_MIN_US, SHOULDER_MAX_US);
  elbowServo.attach(ELBOW_PIN, ELBOW_MIN_US, ELBOW_MAX_US);
  wristRotServo.attach(WRIST_ROT_PIN, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US);
  wristBendServo.attach(WRIST_BEND_PIN, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US);

  // ---- SEND SAME INITIAL POSITION ----
  baseServo.writeMicroseconds(
    degToUs(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG, BASE_MIN_US, BASE_MAX_US));
  shoulderServo.writeMicroseconds(
    degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG, SHOULDER_MIN_US, SHOULDER_MAX_US));
  elbowServo.writeMicroseconds(
    degToUs(elbDeg, ELB_MIN_DEG, ELB_MAX_DEG, ELBOW_MIN_US, ELBOW_MAX_US));
  wristRotServo.writeMicroseconds(
    degToUs(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US));
  wristBendServo.writeMicroseconds(
    degToUs(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US));
  shoulderServo.writeMicroseconds(
  degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG, SHOULDER_MIN_US, SHOULDER_MAX_US));

  Serial.println("=== ARM INITIALIZED TO DEFAULT POSE ===");
}

// ================= LOOP =================
void loop() {
  BP32.update();
  if (!gp || !gp->isConnected()) return;

  int lx = gp->axisX();
  int ly = gp->axisY();
  int rx = gp->axisRX();
  int ry = gp->axisRY();
  bool dpadUp   = gp->dpad() & DPAD_UP;
  bool dpadDown = gp->dpad() & DPAD_DOWN;

  bool changed = false;

  // -------- BASE --------
  if (abs(lx) > DEADZONE) {
    baseDeg -= lx * 0.0025;
    changed = true;
  }

  // -------- SHOULDER --------
  if (abs(ly) > DEADZONE) {
    shoDeg -= ly * 0.0020;
    changed = true;
  }

  // -------- ELBOW --------
  if (abs(ry) > DEADZONE) {
    elbDeg -= ry * 0.0020;
    changed = true;
  }

  // -------- WRIST BEND --------
    if (dpadUp) {
      wristBendDeg += 1.0;
      changed = true;
    }
    else if (dpadDown) {
      wristBendDeg -= 1.0;
      changed = true;
    }

  // -------- WRIST ROT --------
  if (abs(rx) > DEADZONE) {
    wristRotDeg += rx * 0.0020;
    changed = true;
  }

  // -------- CLAMP --------
  baseDeg      = clampf(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG);
  shoDeg       = clampf(shoDeg,  SHO_MIN_DEG,  SHO_MAX_DEG);
  elbDeg       = clampf(elbDeg,  ELB_MIN_DEG,  ELB_MAX_DEG);
  wristRotDeg  = clampf(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG);
  wristBendDeg = clampf(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG);

  // -------- OUTPUT --------
  baseServo.writeMicroseconds(
    degToUs(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG, BASE_MIN_US, BASE_MAX_US));
  shoulderServo.writeMicroseconds(
    degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG, SHOULDER_MIN_US, SHOULDER_MAX_US));
  elbowServo.writeMicroseconds(
    degToUs(elbDeg, ELB_MIN_DEG, ELB_MAX_DEG, ELBOW_MIN_US, ELBOW_MAX_US));
  wristRotServo.writeMicroseconds(
    degToUs(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US));
  wristBendServo.writeMicroseconds(
    degToUs(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US));

  // -------- SERIAL DEBUG --------
  if (changed) {
    Serial.print("BASE: "); Serial.print(baseDeg, 1);
    Serial.print(" | SHO: "); Serial.print(shoDeg, 1);
    Serial.print(" | ELB: "); Serial.print(elbDeg, 1);
    Serial.print(" | WR_ROT: "); Serial.print(wristRotDeg, 1);
    Serial.print(" | WR_BEND: "); Serial.println(wristBendDeg, 1);
  }

  delay(LOOP_DELAY);
}
