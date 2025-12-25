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
#define DEADZONE     40
#define LOOP_DT_MS   10

#define BASE_GAIN    0.0025
#define ARM_GAIN     0.0020
#define WRIST_GAIN   0.0020

#define ACCEL_LIMIT  0.03   // deg/loop² (smoothness)
#define MAX_STEP     0.35   // deg/loop (safety)

// ================= OBJECTS =================
Servo baseServo, shoulderServo, elbowServo;
Servo wristRotServo, wristBendServo;
GamepadPtr gp = nullptr;

// ================= STATE =================
float baseDeg = 135, shoDeg = 90, elbDeg = 90;
float wristRotDeg = 135, wristBendDeg = 90;

// Velocity states
float baseVel = 0, shoVel = 0, elbVel = 0;
float wristRotVel = 0, wristBendVel = 0;

// Filtered joystick
float lxF = 0, lyF = 0, rxF = 0, ryF = 0;

// Wrist lock
bool wristLock = false;
float wristLockRef = 0, shoLockRef = 0, elbLockRef = 0;

// ================= HELPERS =================
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float approach(float cur, float tgt, float step) {
  float d = tgt - cur;
  if (d > step) d = step;
  if (d < -step) d = -step;
  return cur + d;
}

static inline float lpf(float prev, float input, float a) {
  return prev + a * (input - prev);
}

int degToUs(float deg, float dMin, float dMax, int usMin, int usMax) {
  deg = clampf(deg, dMin, dMax);
  float t = (deg - dMin) / (dMax - dMin);
  return (int)lroundf(usMin + t * (usMax - usMin));
}

// ================= CONTROLLER =================
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
  BP32.setup(&onConnected, &onDisconnected);

  baseServo.attach(BASE_PIN, BASE_MIN_US, BASE_MAX_US);
  shoulderServo.attach(SHOULDER_PIN, SHOULDER_MIN_US, SHOULDER_MAX_US);
  elbowServo.attach(ELBOW_PIN, ELBOW_MIN_US, ELBOW_MAX_US);
  wristRotServo.attach(WRIST_ROT_PIN, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US);
  wristBendServo.attach(WRIST_BEND_PIN, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US);
}

// ================= LOOP =================
void loop() {
  static uint32_t last = 0;
  if (millis() - last < LOOP_DT_MS) return;
  last = millis();

  BP32.update();

  int lx = 0, ly = 0, rx = 0, ry = 0;
  bool dpadUp = false, dpadDown = false;
  bool lockBtn = false;

  if (gp && gp->isConnected()) {
    lx = gp->axisX();
    ly = gp->axisY();
    rx = gp->axisRX();
    ry = gp->axisRY();
    lockBtn = gp->buttons() & BUTTON_SHOULDER_L;

    uint8_t dpad = gp->dpad();
    dpadUp   = dpad & DPAD_UP;
    dpadDown = dpad & DPAD_DOWN;
  }

  // ---------- Low-pass filter ----------
  lxF = lpf(lxF, lx, 0.15);
  lyF = lpf(lyF, ly, 0.15);
  rxF = lpf(rxF, rx, 0.15);
  ryF = lpf(ryF, ry, 0.15);

  // ---------- Wrist lock toggle ----------
  static bool prevLock = false;
  if (lockBtn && !prevLock) {
    wristLock = !wristLock;
    wristLockRef = wristBendDeg;
    shoLockRef = shoDeg;
    elbLockRef = elbDeg;
  }
  prevLock = lockBtn;

  // ---------- Velocity targets ----------
  float baseVT = 0, shoVT = 0, elbVT = 0, wrVT = 0, wbVT = 0;

  if (abs(lxF) > DEADZONE) baseVT = -lxF * BASE_GAIN;
  if (abs(lyF) > DEADZONE) shoVT  =  lyF * ARM_GAIN;
  if (abs(ryF) > DEADZONE) elbVT  = -ryF * ARM_GAIN;
  if (abs(rxF) > DEADZONE && !wristLock) wrVT = rxF * WRIST_GAIN;

  if (!wristLock) {
    if (dpadUp)   wbVT =  0.5;
    if (dpadDown) wbVT = -0.5;
  }

  // ---------- Acceleration limiting ----------
  baseVel     = approach(baseVel,     baseVT, ACCEL_LIMIT);
  shoVel      = approach(shoVel,      shoVT,  ACCEL_LIMIT);
  elbVel      = approach(elbVel,      elbVT,  ACCEL_LIMIT);
  wristRotVel = approach(wristRotVel, wrVT,   ACCEL_LIMIT);
  wristBendVel= approach(wristBendVel,wbVT,   ACCEL_LIMIT);

  // ---------- Integrate ----------
  baseDeg      += baseVel;
  shoDeg       += shoVel;
  elbDeg       += elbVel;
  wristRotDeg  += wristRotVel;
  wristBendDeg += wristBendVel;

  // ---------- Wrist lock compensation ----------
  if (wristLock) {
    wristBendDeg = wristLockRef + (shoDeg - shoLockRef) + (elbDeg - elbLockRef);
  }

  // ---------- Clamp ----------
  baseDeg      = clampf(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG);
  shoDeg       = clampf(shoDeg,  SHO_MIN_DEG,  SHO_MAX_DEG);
  elbDeg       = clampf(elbDeg,  ELB_MIN_DEG,  ELB_MAX_DEG);
  wristRotDeg  = clampf(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG);
  wristBendDeg = clampf(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG);

  // ---------- Output ----------
  baseServo.writeMicroseconds(degToUs(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG, BASE_MIN_US, BASE_MAX_US));
  shoulderServo.writeMicroseconds(degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG, SHOULDER_MIN_US, SHOULDER_MAX_US));
  elbowServo.writeMicroseconds(degToUs(elbDeg, ELB_MIN_DEG, ELB_MAX_DEG, ELBOW_MIN_US, ELBOW_MAX_US));
  wristRotServo.writeMicroseconds(degToUs(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US));
  wristBendServo.writeMicroseconds(degToUs(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US));
}
  