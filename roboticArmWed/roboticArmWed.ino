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
#define BASE_MIN_US        700 //500
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
#define BASE_MIN_DEG        0
#define BASE_MAX_DEG       270
#define SHO_MIN_DEG         0
#define SHO_MAX_DEG        270
#define ELB_MIN_DEG         0
#define ELB_MAX_DEG        270
#define WRIST_ROT_MIN_DEG   0
#define WRIST_ROT_MAX_DEG  270
#define WRIST_BEND_MIN_DEG  0
#define WRIST_BEND_MAX_DEG 270

// ================= CONTROL =================
#define DEADZONE     40
#define LOOP_DT_MS   10

#define BASE_GAIN    0.0012
#define ARM_GAIN     0.0010
#define WRIST_GAIN   0.0009

#define ACCEL_LIMIT  0.012   // deg/loop² (smoothness)
#define MAX_STEP     0.35   // deg/loop (safety)

// ================= OBJECTS =================
Servo baseServo, shoulderServo, elbowServo;
Servo wristRotServo, wristBendServo;
GamepadPtr gp = nullptr;

// ================= STATE =================
float baseDeg = 150, shoDeg = 125, elbDeg = 20;
float wristRotDeg = 0, wristBendDeg = 120;

// Velocity states
float baseStep = 0, shoStep = 0, elbStep = 0;
float wristRotStep = 0, wristBendStep = 0;

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
  //float d = tgt - cur;
  if(abs(d) < step) return cur;
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

// Map base angle -> allowed max shoulder angle (piecewise linear)
// float shoulderMaxForBase(float baseDeg) {
//   // Clamp base to the defined range [0, 260]
//   float b = clampf(baseDeg, 0.0f, 260.0f);

//   // Segment 0..45 : 40 -> 60
//   if (b <= 45.0f) {
//     float t = (b - 0.0f) / (45.0f - 0.0f);
//     return 40.0f + t * (60.0f - 40.0f);
//   }

//   // Segment 45..90 : 60 -> 90
//   if (b <= 90.0f) {
//     float t = (b - 45.0f) / (90.0f - 45.0f);
//     return 60.0f + t * (90.0f - 60.0f);
//   }

//   // Segment 90..135 : 90 -> 120
//   float t = (b - 90.0f) / (135.0f - 90.0f);
//   return 90.0f + t * (120.0f - 90.0f);
// }

  float shoulderMaxForBase(float baseDeg) {

    if (baseDeg < 22.5f) {          // around 0°
      return 40.0f;
    }
    else if (baseDeg < 67.5f) {     // around 45°
      return 60.0f;
    }
    else if (baseDeg < 112.5f) {    // around 90°
      return 90.0f;
    }
    else {                          // around 135° and above
      return 120.0f;
    }
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
  float baseJoystick = 0, shoJoystick = 0, elbJoystick = 0, wrVT = 0, wbVT = 0;

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
  lxF = lpf(lxF, lx, 0.10);
  lyF = lpf(lyF, ly, 0.1);
  rxF = lpf(rxF, rx, 0.1);
  ryF = lpf(ryF, ry, 0.10);

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

  if (abs(lxF) > DEADZONE) baseJoystick = +lxF * BASE_GAIN;
  if (abs(lyF) > DEADZONE) shoJoystick  = -lyF * ARM_GAIN;
  if (abs(ryF) > DEADZONE) elbJoystick  = +ryF * ARM_GAIN;
  if (abs(rxF) > DEADZONE && !wristLock) wrVT = rxF * WRIST_GAIN;

  if (!wristLock) {
    if (dpadUp)   wbVT =  0.5;
    if (dpadDown) wbVT = -0.5;
  }

    // ---------- Acceleration limiting ----------
  baseStep     = approach(baseStep,     baseJoystick, ACCEL_LIMIT);
  shoStep      = approach(shoStep,      shoJoystick,  ACCEL_LIMIT);
  elbStep      = approach(elbStep,      elbJoystick,  ACCEL_LIMIT);
  wristRotStep = approach(wristRotStep, wrVT,         ACCEL_LIMIT);
  wristBendStep= approach(wristBendStep,wbVT,         ACCEL_LIMIT);

  // ============================================================
  // 1) PREDICT NEXT ANGLES (do NOT commit yet)
  // ============================================================
  float baseNext      = baseDeg + baseStep;
  float shoNext       = shoDeg  + shoStep;
  float elbNext       = elbDeg  + elbStep;
  float wristRotNext  = wristRotDeg  + wristRotStep;
  float wristBendNext = wristBendDeg + wristBendStep;

  // Clamp baseNext first (so shoulderMinAllowed uses a valid base)
  baseNext = clampf(baseNext, BASE_MIN_DEG, BASE_MAX_DEG);

  // ============================================================
  // 2) BASE SAFETY: reject base motion if it would make shoulder illegal
  //    (i.e., shoulder would be above its allowed "top" at that base angle)
  // ============================================================
  float shoMinAllowed_nextBase = shoulderMaxForBase(baseNext);
  shoMinAllowed_nextBase = clampf(shoMinAllowed_nextBase, SHO_MIN_DEG, SHO_MAX_DEG);

  // If moving base would require shoulder to be >= bigger minimum than we currently have,
  // then that base move would put shoulder in an illegal position -> freeze base.
  if (shoDeg < shoMinAllowed_nextBase) {
    baseNext = baseDeg;     // don't move base
    baseStep = 0.0f;        // stop base velocity too (prevents pushing against the limit)
  }

  // ============================================================
  // 3) Now apply the chosen next angles (base may have been frozen)
  // ============================================================
  baseDeg      = baseNext;
  shoDeg       = shoNext;
  elbDeg       = elbNext;
  wristRotDeg  = wristRotNext;
  wristBendDeg = wristBendNext;

  // ---------- Wrist lock compensation ----------
  if (wristLock) {
    wristBendDeg = wristLockRef
                 + 0.75f*(shoDeg - shoLockRef)
                 - 0.70f*(elbDeg - elbLockRef);
  }

  // ---------- Clamp (basic) ----------
  baseDeg      = clampf(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG);
  elbDeg       = clampf(elbDeg,  ELB_MIN_DEG,  ELB_MAX_DEG);
  wristRotDeg  = clampf(wristRotDeg,  WRIST_ROT_MIN_DEG,  WRIST_ROT_MAX_DEG);
  wristBendDeg = clampf(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG);

  // ============================================================
  // 4) Shoulder clamp using the FINAL baseDeg (after possible freeze)
  // ============================================================
  float shoMinAllowed = shoulderMaxForBase(baseDeg);
  shoMinAllowed = clampf(shoMinAllowed, SHO_MIN_DEG, SHO_MAX_DEG);

  // Hard stop at the dynamic "top" (lifting up)
  if (shoDeg <= shoMinAllowed && shoStep < 0.0f) {
    shoDeg = shoMinAllowed;
    shoStep = 0.0f;
  }

  // Hard stop at physical bottom (moving down)
  if (shoDeg >= SHO_MAX_DEG && shoStep > 0.0f) {
    shoDeg = SHO_MAX_DEG;
    shoStep = 0.0f;
  }

  // ---------- Clamp ----------
  baseDeg      = clampf(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG);
  //shoDeg       = clampf(shoDeg,  SHO_MIN_DEG,  SHO_MAX_DEG);
  elbDeg       = clampf(elbDeg,  ELB_MIN_DEG,  ELB_MAX_DEG);
  wristRotDeg  = clampf(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG);
  wristBendDeg = clampf(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG);

  // Dynamic shoulder max depending on base angle
  // float shoMaxDyn = shoulderMaxForBase(baseDeg);
  // shoMaxDyn = clampf(shoMaxDyn, SHO_MIN_DEG, SHO_MAX_DEG); // safety with your global limits
  // shoDeg = clampf(shoDeg, SHO_MIN_DEG, shoMaxDyn);

//float shoMax = shoulderMaxForBase(baseDeg);

// HARD STOP — instant
// if (shoDeg < shoMax) {
//   shoDeg = shoMax;


// Hard stop at physical bottom (moving down)
if (shoDeg >= SHO_MAX_DEG && shoStep > 0.0f) {
  shoDeg = SHO_MAX_DEG;
  shoStep = 0.0f;
}

  // ---------- Output ----------
  baseServo.writeMicroseconds(degToUs(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG, BASE_MIN_US, BASE_MAX_US));
  shoulderServo.writeMicroseconds(degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG, SHOULDER_MIN_US, SHOULDER_MAX_US));
  elbowServo.writeMicroseconds(degToUs(elbDeg, ELB_MIN_DEG, ELB_MAX_DEG, ELBOW_MIN_US, ELBOW_MAX_US));
  wristRotServo.writeMicroseconds(degToUs(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US));
  wristBendServo.writeMicroseconds(degToUs(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US));
}
  