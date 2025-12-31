// base: 185
// Sho: 155
// Elb: 20
// wristBend: 120
// tip: 135
// more control closer to the center of the joystich
// --> lrx * by same term but with absolute value

//DONE: slower speed for elbow  

// DONE: vertical position of the shoulder instead of 125 put it 120
// Done: move tip control to the Dpad 
// Done: slow down the speed for all servos
// delay 

// to do:
// fix starting position of the wrist: 
// faster servos


#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <math.h>


// ================= PINS =================
#define BASE_PIN        27
#define SHOULDER_PIN    14
#define ELBOW_PIN       12
#define WRIST_ROT_PIN   16
#define WRIST_BEND_PIN  13
#define RELAY_PIN       32 // NEW 

// ================= SERVO LIMITS (µs) =================
#define BASE_MIN_US        500 //500
#define BASE_MAX_US       2500
#define SHOULDER_MIN_US    500
#define SHOULDER_MAX_US   2500
#define ELBOW_MIN_US       500
#define ELBOW_MAX_US      2500
#define WRIST_ROT_MIN_US   500
#define WRIST_ROT_MAX_US  2500
#define WRIST_BEND_MIN_US  500
#define WRIST_BEND_MAX_US 1833

// ================= ANGLE LIMITS (deg) =================
#define BASE_MIN_DEG        0
#define BASE_MAX_DEG       270
#define SHO_MIN_DEG         0
#define SHO_MAX_DEG        270
#define ELB_MIN_DEG         0
#define ELB_MAX_DEG        270
#define WRIST_ROT_MIN_DEG   0
#define WRIST_ROT_MAX_DEG  270
#define WRIST_BEND_MIN_DEG  30
#define WRIST_BEND_MAX_DEG 180

// ================= CONTROL =================
#define DEADZONE     40
#define LOOP_DT_MS   10 // controls the loop

// #define BASE_GAIN   0.0012
// #define ARM_GAIN    0.0010
// #define WRIST_GAIN  0.0009

#define BASE_GAIN    0.0008
#define ARM_GAIN     0.0005
#define WRIST_GAIN   0.00045

#define ACCEL_LIMIT  0.012   // deg/loop² (smoothness)
//#define MAX_STEP     0.35   // deg/loop (safety)
#define MAX_STEP     0.20

#define SHO_SOFT_ZONE_DEG   30.0f   // start slowing 30° before limit
#define SHO_SOFT_GAIN      0.035f  // deg/loop per deg of distance

// ================= OBJECTS =================
Servo baseServo, shoulderServo, elbowServo;
Servo wristRotServo, wristBendServo;
GamepadPtr gp = nullptr;

// ================= STATE =================
float baseDeg = 185, shoDeg = 155, elbDeg = 20;
float wristRotDeg = 135, wristBendDeg = 120;

// Velocity states
float baseStep = 0, shoStep = 0, elbStep = 0;
float wristRotStep = 0, wristBendStep = 0;

// Filtered joystick
float lxF = 0, lyF = 0, ryF = 0;
//float rxF = 0

// Wrist lock
bool wristLock = false;
float wristLockRef = 0, shoLockRef = 0, elbLockRef = 0;

// ================= HELPERS =================
static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline float approach(float cur, float tgt, float step) {
  float d = tgt - cur;
  //if(fabs(d) < step) return cur;
  if (d > step) d = step;
  if (d < -step) d = -step;
  return cur + d; // cur +/- step
}

static inline float lpf(float prev, float input, float a) { 
  return prev + a * (input - prev);
}

int degToUs(float deg, float dMin, float dMax, int usMin, int usMax) {
  deg = clampf(deg, dMin, dMax);
  float t = (deg - dMin) / (dMax - dMin);
  return (int)lroundf(usMin + t * (usMax - usMin));
}

    // if (baseDeg < 22.5f) {          // around 0°
    //   return 40.0f;
    // }
    // else if (baseDeg < 67.5f) {     // around 45°
    //   return 60.0f;
    // }
    // else if (baseDeg < 112.5f) {    // around 90°
    //   return 90.0f;
    // }
    // else {                          // around 135° and above
    //   return 120.0f;
    // }
  

  float shoulderMaxForBase(float baseDeg) {

  // Clamp base range (safety)
  if (baseDeg < 0.0f)   baseDeg = 0.0f;
  if (baseDeg > 270.0f) baseDeg = 270.0f;

  // -------- Front region (0 → 10) --------
  if (baseDeg <= 10.0f) {
    // (0,215) → (10,220)
    return 215.0f + (baseDeg - 0.0f) * (220.0f - 215.0f) / (10.0f - 0.0f);
  }

  // -------- Transition up (10 → 20) --------
  else if (baseDeg <= 20.0f) {
    // (10,220) → (20,230)
    return 220.0f + (baseDeg - 10.0f) * (230.0f - 220.0f) / (20.0f - 10.0f);
  }

  // -------- Safe flat zone (20 → 230) --------
  else if (baseDeg <= 230.0f) {
    return 230.0f;
  }

  // -------- Rear transition down --------
  else if (baseDeg <= 240.0f) {
    // (230,230) → (240,225)
    return 230.0f + (baseDeg - 230.0f) * (225.0f - 230.0f) / (240.0f - 230.0f);
  }
  else if (baseDeg <= 250.0f) {
    // (240,225) → (250,210)
    return 225.0f + (baseDeg - 240.0f) * (210.0f - 225.0f) / (250.0f - 240.0f);
  }
  else if (baseDeg <= 260.0f) {
    // (250,210) → (260,197)
    return 210.0f + (baseDeg - 250.0f) * (197.0f - 210.0f) / (260.0f - 250.0f);
  }
  else {
    // (260,197) → (270,180)
    return 197.0f + (baseDeg - 260.0f) * (180.0f - 197.0f) / (270.0f - 260.0f);
  }
}

float shoulderMinForBase(float baseDeg) {
  // Clamp base into [0, 270]
  baseDeg = clampf(baseDeg, 0.0f, 270.0f);

  // Flat zones
  if (baseDeg <= 50.0f)  return 40.0f;   // 0..50
  if (baseDeg >= 200.0f) return 40.0f;   // 200..270

  // Linear segments between given points:
  // (50,40) -> (60,48)
  if (baseDeg <= 60.0f)
    return 40.0f + (baseDeg - 50.0f) * (48.0f - 40.0f) / (60.0f - 50.0f);

  // (60,48) -> (70,60)
  if (baseDeg <= 70.0f)
    return 48.0f + (baseDeg - 60.0f) * (60.0f - 48.0f) / (70.0f - 60.0f);

  // (70,60) -> (80,73)
  if (baseDeg <= 80.0f)
    return 60.0f + (baseDeg - 70.0f) * (73.0f - 60.0f) / (80.0f - 70.0f);

  // (80,73) -> (90,86)
  if (baseDeg <= 90.0f)
    return 73.0f + (baseDeg - 80.0f) * (86.0f - 73.0f) / (90.0f - 80.0f);

  // (90,86) -> (95,113)
  if (baseDeg <= 95.0f)
    return 86.0f + (baseDeg - 90.0f) * (113.0f - 86.0f) / (95.0f - 90.0f);

  // (95,113) -> (100,120)
  if (baseDeg <= 100.0f)
    return 113.0f + (baseDeg - 95.0f) * (120.0f - 113.0f) / (100.0f - 95.0f);

  // (100 → 125) flat at 120
  if (baseDeg <= 125.0f)
    return 115.0f; //changed

  // (125,125) -> (145,125) flat
  if (baseDeg <= 145.0f) return 125.0f; //here

  // (145,125) -> (150,120)
  if (baseDeg <= 150.0f)
    return 125.0f + (baseDeg - 145.0f) * (120.0f - 125.0f) / (150.0f - 145.0f);

  // (150,120) -> (155,110)
  if (baseDeg <= 155.0f)
    return 120.0f + (baseDeg - 150.0f) * (110.0f - 120.0f) / (155.0f - 150.0f);

  // (155,110) -> (160,85)
  if (baseDeg <= 160.0f)
    return 110.0f + (baseDeg - 155.0f) * (85.0f - 110.0f) / (160.0f - 155.0f);

  // (160,85) -> (170,75)
  if (baseDeg <= 170.0f)
    return 85.0f + (baseDeg - 160.0f) * (75.0f - 85.0f) / (170.0f - 160.0f);

  // (170,75) -> (180,60)
  if (baseDeg <= 180.0f)
    return 75.0f + (baseDeg - 170.0f) * (60.0f - 75.0f) / (180.0f - 170.0f);

  // (180,60) -> (190,45)
  if (baseDeg <= 190.0f)
    return 60.0f + (baseDeg - 180.0f) * (45.0f - 60.0f) / (190.0f - 180.0f);

  // (190,45) -> (200,40)
  return 45.0f + (baseDeg - 190.0f) * (40.0f - 45.0f) / (200.0f - 190.0f);
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
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Relay OFF at boot

  baseServo.attach(BASE_PIN, BASE_MIN_US, BASE_MAX_US);
  shoulderServo.attach(SHOULDER_PIN, SHOULDER_MIN_US, SHOULDER_MAX_US);
  elbowServo.attach(ELBOW_PIN, ELBOW_MIN_US, ELBOW_MAX_US);
  wristRotServo.attach(WRIST_ROT_PIN, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US);
  wristBendServo.attach(WRIST_BEND_PIN, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US);
}

// ================= LOOP =================
void loop() {

  //digitalWrite(RELAY_PIN, HIGH);

  static uint32_t last = 0;
  if (millis() - last < LOOP_DT_MS) return;
  last = millis();

  BP32.update();

  // ================= RELAY SAFETY =================
  static bool relayOn = false;

  //digitalWrite(RELAY_PIN, HIGH);

  if (gp && gp->isConnected()) {
    if (!relayOn) {
      digitalWrite(RELAY_PIN, HIGH);
      relayOn = true;
    }
  } else {
    if (relayOn) {
      digitalWrite(RELAY_PIN, LOW);
      relayOn = false;
    }
  }

  int lx = 0, ly = 0, rx = 0, ry = 0;
  bool dpadUp = false, dpadDown = false;
  bool dpadRight = false, dpadLeft = false;
  bool lockBtn = false;

  float baseJoystick = 0;
  float shoJoystick  = 0;
  float elbJoystick  = 0;
  float wrVT = 0, wbVT = 0;

  // ================= READ CONTROLLER =================
  if (gp && gp->isConnected()) {
    lx = gp->axisX();
    ly = gp->axisY();
    //rx = gp->axisRX();
    ry = gp->axisRY();
    lockBtn = gp->buttons() & BUTTON_SHOULDER_L;

    uint8_t dpad = gp->dpad();
    dpadUp   = dpad & DPAD_UP;
    dpadDown = dpad & DPAD_DOWN;
    dpadRight = dpad & DPAD_RIGHT;
    dpadLeft = dpad & DPAD_LEFT;
  }

  // ================= FILTER =================
  lxF = lpf(lxF, lx, 0.10f);
  lyF = lpf(lyF, ly, 0.10f);
  //rxF = lpf(rxF, rx, 0.10f);
  ryF = lpf(ryF, ry, 0.10f);

  // ================= WRIST LOCK TOGGLE =================
  static bool prevLock = false;
  if (lockBtn && !prevLock) {
    wristLock = !wristLock;
    wristLockRef = wristBendDeg;
    shoLockRef   = shoDeg;
    elbLockRef   = elbDeg;
  }
  prevLock = lockBtn;

  // ================= VELOCITY TARGETS =================
  // if (fabs(lxF) > DEADZONE) baseJoystick =  lxF * BASE_GAIN;
  // if (fabs(lyF) > DEADZONE) shoJoystick  = -lyF * ARM_GAIN;
  // float ax = fabs(lxF);
  // float ay = fabs(lyF);

  // if (ax > DEADZONE || ay > DEADZONE) {

  //   // Axis dominance: choose intent
  //   if (ax > ay) {
  //     // X dominates → base only
  //     baseJoystick = -lxF * abs(lxF) * BASE_GAIN;
  //     shoJoystick  = 0.0f;
  //   } else {
  //     // Y dominates → shoulder only
  //     baseJoystick = 0.0f;
  //     shoJoystick  = -lyF * abs(lyF) * ARM_GAIN;
  //   }
  // }

  float ax = fabs(lxF);
  float ay = fabs(lyF);

  if (ax > DEADZONE || ay > DEADZONE) {
    if (ax > ay) {
      // X dominates → base
      baseJoystick = -lxF * abs(lxF) * BASE_GAIN;
    } else {
      // Y dominates → shoulder
      shoJoystick  = -lyF * abs(lyF) * ARM_GAIN;
    }
  }


  if (fabs(ryF) > DEADZONE) 
    elbJoystick  =  ryF * abs(ryF) *  ARM_GAIN;
  
  // if (fabs(rxF) > DEADZONE && !wristLock) 
    //wrVT = rxF * abs(rxF) * WRIST_GAIN; // to be removed

  if (!wristLock) {
    if (dpadUp)   wbVT =  0.5f;
    if (dpadDown) wbVT = -0.5f;
  }

  if (dpadRight) wrVT =  WRIST_GAIN * 200.0f;
  if (dpadLeft)  wrVT = - WRIST_GAIN * 200.0f;

  // ================= ACCEL LIMIT =================
  baseStep      = approach(baseStep,      baseJoystick, ACCEL_LIMIT);
  shoStep       = approach(shoStep,       shoJoystick,  ACCEL_LIMIT);
  elbStep       = approach(elbStep,       elbJoystick,  ACCEL_LIMIT);
  wristRotStep  = approach(wristRotStep,  wrVT,         ACCEL_LIMIT);
  wristBendStep = approach(wristBendStep, wbVT,         ACCEL_LIMIT);

  // ================= STEP CLAMP =================
  baseStep      = clampf(baseStep,      -MAX_STEP, MAX_STEP);
  shoStep       = clampf(shoStep,       -MAX_STEP, MAX_STEP);
  elbStep       = clampf(elbStep,       -MAX_STEP, MAX_STEP);
  wristRotStep  = clampf(wristRotStep,  -MAX_STEP, MAX_STEP);
  wristBendStep = clampf(wristBendStep, -MAX_STEP, MAX_STEP);

    // Predict base
  float baseNext = clampf(baseDeg + baseStep, BASE_MIN_DEG, BASE_MAX_DEG);

  // Dynamic shoulder limits
  float shoMinAllowed =
    clampf(shoulderMinForBase(baseNext), SHO_MIN_DEG, SHO_MAX_DEG);

  float shoMaxAllowed =
    clampf(shoulderMaxForBase(baseNext), SHO_MIN_DEG, SHO_MAX_DEG);

  // Soft zone toward max
  if (shoStep > 0.0f) {
    float dist = shoMaxAllowed - shoDeg;
    if (dist < 0.0f) dist = 0.0f;
    if (dist < SHO_SOFT_ZONE_DEG) {
      float maxStep = SHO_SOFT_GAIN * dist;
      if (shoStep > maxStep) shoStep = maxStep;
    }
  }

  // Predict shoulder
  float shoNext = shoDeg + shoStep;

  // Enforce min / max
  if (shoNext < shoMinAllowed) {
    shoNext = shoMinAllowed;
    shoStep = 0.0f;
  }
  if (shoNext > shoMaxAllowed) {
    shoNext = shoMaxAllowed;
    shoStep = 0.0f;
  }

  // Freeze base if it would violate shoulder
  if ((shoNext <= shoMinAllowed || shoNext >= shoMaxAllowed) &&
      baseStep != 0.0f) {
    baseNext = baseDeg;
    baseStep = 0.0f;
  }

  // Commit
  baseDeg = baseNext;
  shoDeg  = shoNext;

  // ============================================================
  // OTHER JOINTS
  // ============================================================
  elbDeg       = clampf(elbDeg + elbStep, ELB_MIN_DEG, ELB_MAX_DEG);
  wristRotDeg  = clampf(wristRotDeg + wristRotStep,
                         WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG);
  wristBendDeg = clampf(wristBendDeg + wristBendStep,
                         WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG);

  // ================= WRIST LOCK COMP =================
  if (wristLock) {
    wristBendDeg = wristLockRef
                 + 0.75f * (shoDeg - shoLockRef)
                 - 0.70f * (elbDeg - elbLockRef);

    // wristBendDeg = wristLockRef
    //              + (shoDeg - shoLockRef)
    //              - (elbDeg - elbLockRef);

    wristBendDeg = clampf(
      wristBendDeg,
      WRIST_BEND_MIN_DEG,
      WRIST_BEND_MAX_DEG
    );
  }

  // ================= OUTPUT =================
  baseServo.writeMicroseconds(
    degToUs(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG, BASE_MIN_US, BASE_MAX_US));

  shoulderServo.writeMicroseconds(
    degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG,
            SHOULDER_MIN_US, SHOULDER_MAX_US));

  elbowServo.writeMicroseconds(
    degToUs(elbDeg, ELB_MIN_DEG, ELB_MAX_DEG,
            ELBOW_MIN_US, ELBOW_MAX_US));

  wristRotServo.writeMicroseconds(
    degToUs(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG,
            WRIST_ROT_MIN_US, WRIST_ROT_MAX_US));

  wristBendServo.writeMicroseconds(
    degToUs(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG,
            WRIST_BEND_MIN_US, WRIST_BEND_MAX_US));
}



