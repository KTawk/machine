#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <math.h>

// ======================================================
// PINS – SUCTION (pompe + valve). done
// ======================================================
#define PIN_POMPE 1 //Rx
#define PIN_VALVE 3 //TX
#define STBY 33
// TX (Transmit): GPIO 1
// RX (Receive): GPIO 3

// ======================================================
// PINS – MECANUM / OMNI DRIVE done
// ======================================================
#define UL1 18
#define UL2 19
#define UR1 2
#define UR2 15
#define DL1 5
#define DL2 17
#define DR1 22
#define DR2 21

// ======================================================
// PINS – LINEAR ACTUATOR (L298N / pont en H) done
// ======================================================
#define ACT_IN1 25
#define ACT_IN2 26

// ======================================================
// PINS – ARM done
// ======================================================
#define BASE_PIN        27 //done
#define SHOULDER_PIN    32 //done
#define ELBOW_PIN       12 //done
#define WRIST_ROT_PIN   14 //done
#define WRIST_BEND_PIN  13 //done

// ======================================================
// RELAY - done
// ======================================================
#define MAIN_RELAY_PIN    35 
#define ALTERNATIVE_RELAY_PIN  34 

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

// Relay:
int valveCounter = 0;
bool valveTriggered = false;

// ======================================================
// PARAMS
// ======================================================
#define DEADZONE 40
//#define MAX_PWM  255

// ======================================================
// SUCTION
// ======================================================
bool suctionOn = false;
bool prevR1 = false;
//uint8_t r1PressCount = 0;

// ======================================================
// MODE CONTROL 
// ======================================================
enum PowerSource {
  POWER_NONE,
  MAIN_POWER,
  POWER_SAFETY
};
PowerSource activePower = POWER_NONE;

enum ControlMode {
  MODE_DRIVE,
  MODE_ARM
};
ControlMode currentMode = MODE_DRIVE;

// Circle button toggle
bool prevCircle = false;
bool prevTriangle = false;

bool valvePulsing = false;
uint32_t valvePulseStartMs = 0;
const uint32_t VALVE_PULSE_MS = 100;

// ======================================================
// MOTOR CONTROL (mecanum)
// ======================================================
void setMotor(int in1, int in2, float pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(in1, (int)pwm);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, (int)(-pwm));
  }
}

void stopAllDrive() {
  setMotor(UL1, UL2, 0);
  setMotor(UR1, UR2, 0);
  setMotor(DL1, DL2, 0);
  setMotor(DR1, DR2, 0);
}

// ======================================================
// OMNI DRIVE + SPIN
// ======================================================
void omniDrive(int joyX, int joyY, int joyRX) {
  if (abs(joyX) < DEADZONE) joyX = 0;
  if (abs(joyY) < DEADZONE) joyY = 0;
  if (abs(joyRX) < DEADZONE) joyRX = 0;

  if (joyX == 0 && joyY == 0 && joyRX == 0) {
    stopAllDrive();
    return;
  }

  float X = -joyX / 509.0f;
  float Y =  joyY / 509.0f;

  float angle = atan2f(Y, X);
  float R = sqrtf(X * X + Y * Y);
  float speed = R * R * 255.0f;

  float ul = cosf(angle - PI / 4)      * speed;
  float ur = cosf(angle + PI / 4)      * speed;
  float dl = cosf(angle - 3 * PI / 4)  * speed;
  float dr = cosf(angle + 3 * PI / 4)  * speed;

  const float SPIN_GAIN = 0.8f;
  float spin = -(joyRX / 509.0f) * 255.0f * SPIN_GAIN;

  ul += spin;
  ur -= spin;
  dl += spin;
  dr -= spin;

  float maxv = max(max(fabs(ul), fabs(ur)), max(fabs(dl), fabs(dr)));
  if (maxv > 255.0f) {
    float k = 255.0f / maxv;
    ul *= k; ur *= k; dl *= k; dr *= k;
  }

  setMotor(UL1, UL2, ul);
  setMotor(UR1, UR2, ur);
  setMotor(DL1, DL2, dl);
  setMotor(DR1, DR2, dr);
}

// ======================================================
// ACTUATOR CONTROL (DPAD UP / DOWN)
// ======================================================
void actuatorStop() {
  digitalWrite(ACT_IN1, LOW);
  digitalWrite(ACT_IN2, LOW);
}

void actuatorForward() {
  digitalWrite(ACT_IN1, HIGH);
  digitalWrite(ACT_IN2, LOW);
}

void actuatorReverse() {
  digitalWrite(ACT_IN1, LOW);
  digitalWrite(ACT_IN2, HIGH);
}

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

void handleDriveMode() {
  // ---- DRIVE ----
  omniDrive(gp->axisX(), gp->axisY(), gp->axisRX());

  // ---- ACTUATOR (DPAD) ----
  uint8_t dpad = gp->dpad();

  if (dpad & DPAD_UP) {
    actuatorForward();
  } else if (dpad & DPAD_DOWN) {
    actuatorReverse();
  } else {
    actuatorStop();
  }
}

void handleArmMode() {
  static uint32_t last = 0;
  if (millis() - last < LOOP_DT_MS) return;
  last = millis();

  // --- READ CONTROLLER ---
  int lx = gp->axisX();
  int ly = gp->axisY();
  int ry = gp->axisRY();

  uint8_t dpad = gp->dpad();
  bool dpadUp    = dpad & DPAD_UP;
  bool dpadDown  = dpad & DPAD_DOWN;
  bool dpadRight = dpad & DPAD_RIGHT;
  bool dpadLeft  = dpad & DPAD_LEFT;

  bool lockBtn = gp->buttons() & BUTTON_SHOULDER_L;

  // ================= FILTER =================
  float baseJoystick = 0.0f;
  float shoJoystick  = 0.0f;
  float elbJoystick  = 0.0f;
  float wrVT = 0.0f;
  float wbVT = 0.0f;

  lxF = lpf(lxF, lx, 0.10f);
  lyF = lpf(lyF, ly, 0.10f);
  ryF = lpf(ryF, ry, 0.10f);

  if (fabs(lxF) < DEADZONE) lxF = 0;
  if (fabs(lyF) < DEADZONE) lyF = 0;
  if (fabs(ryF) < DEADZONE) ryF = 0;

  // ================= WRIST LOCK TOGGLE =================
  static bool prevLock = false;
  if (lockBtn && !prevLock) {
    wristLock = !wristLock;
    wristLockRef = wristBendDeg;
    shoLockRef   = shoDeg;
    elbLockRef   = elbDeg;
  }
  prevLock = lockBtn;

  float ax = fabs(lxF);
  float ay = fabs(lyF);

  if (ax > DEADZONE || ay > DEADZONE) {
    if (ax > ay) {
      // X dominates → base
      baseJoystick = -lxF * fabs(lxF) * BASE_GAIN;
    } else {
      // Y dominates → shoulder
      shoJoystick  = -lyF * fabs(lyF) * ARM_GAIN;
    }
  }


  if (fabs(ryF) > DEADZONE) 
    elbJoystick  =  ryF * fabs(ryF) *  ARM_GAIN;

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

void armStop() {
  // Stop motion immediately
  baseStep = 0.0f;
  shoStep  = 0.0f;
  elbStep  = 0.0f;
  wristRotStep  = 0.0f;
  wristBendStep = 0.0f;

  // Detach servos (remove torque & PWM)
  if (baseServo.attached())      baseServo.detach();
  if (shoulderServo.attached())  shoulderServo.detach();
  if (elbowServo.attached())     elbowServo.detach();
  if (wristRotServo.attached())  wristRotServo.detach();
  if (wristBendServo.attached()) wristBendServo.detach();

  Serial.println("ARM STOP");
}

// void taskValvePulse() {
//   if (!valvePulsing) return;

//   if (millis() - valvePulseStartMs >= VALVE_PULSE_MS) {
//     digitalWrite(PIN_VALVE, LOW);
//     valvePulsing = false;
//   }
// }

void armAttach() {
  if (!baseServo.attached())
    baseServo.attach(BASE_PIN, BASE_MIN_US, BASE_MAX_US);

  if (!shoulderServo.attached())
    shoulderServo.attach(SHOULDER_PIN, SHOULDER_MIN_US, SHOULDER_MAX_US);

  if (!elbowServo.attached())
    elbowServo.attach(ELBOW_PIN, ELBOW_MIN_US, ELBOW_MAX_US);

  if (!wristRotServo.attached())
    wristRotServo.attach(WRIST_ROT_PIN, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US);

  if (!wristBendServo.attached())
    wristBendServo.attach(WRIST_BEND_PIN, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US);

  Serial.println("ARM ATTACHED");
}

void setSystemPower(PowerSource next) {
  // 1) Make shoulder limp FIRST
  armStop();

  // 2) Turn BOTH relays OFF (guaranteed safe)
  digitalWrite(MAIN_RELAY_PIN, LOW);
  digitalWrite(ALTERNATIVE_RELAY_PIN, LOW);

  // 3) Small dead-time so relays can physically release
  delay(50);   // intentional and correct for relays

  // 4) Turn ON exactly ONE relay
  if (next == MAIN_POWER
  ) {
    digitalWrite(MAIN_RELAY_PIN, HIGH);
  }
  else if (next == POWER_SAFETY){
    digitalWrite(ALTERNATIVE_RELAY_PIN, HIGH);
  }
  // else POWER_NONE → both stay OFF
  armAttach(); 

  activePower = next;
}


// ======================================================
// GAMEPAD CALLBACKS
// ======================================================
void onConnectedGamepad(GamepadPtr gpRef) {
  gp = gpRef;

  //(true);   

  Serial.println("Controller connected");
}


void onDisconnectedGamepad(GamepadPtr ctl) {
  if (gp == ctl) {
    gp = nullptr;

    stopAllDrive();
    actuatorStop();

    suctionOn = false;
    digitalWrite(PIN_POMPE, LOW);
    digitalWrite(PIN_VALVE, LOW);

    digitalWrite(STBY, LOW);

    Serial.println("Controller disconnected!");
  }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(UL1, OUTPUT); pinMode(UL2, OUTPUT);
  pinMode(UR1, OUTPUT); pinMode(UR2, OUTPUT);
  pinMode(DL1, OUTPUT); pinMode(DL2, OUTPUT);
  pinMode(DR1, OUTPUT); pinMode(DR2, OUTPUT);

  pinMode(PIN_POMPE, OUTPUT);
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_POMPE, LOW);
  digitalWrite(PIN_VALVE, LOW);

  pinMode(ACT_IN1, OUTPUT);
  pinMode(ACT_IN2, OUTPUT);
  actuatorStop();

  // Relays (if present)
  pinMode(MAIN_RELAY_PIN, OUTPUT);
  pinMode(ALTERNATIVE_RELAY_PIN, OUTPUT);
  digitalWrite(MAIN_RELAY_PIN, LOW);
  digitalWrite(ALTERNATIVE_RELAY_PIN, LOW);
  activePower = POWER_NONE;

  // Bluetooth
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  // BP32.forgetBluetoothKeys(); // enable only during dev

  // Attach servos ONCE
  baseServo.attach(BASE_PIN, BASE_MIN_US, BASE_MAX_US);
  shoulderServo.attach(SHOULDER_PIN, SHOULDER_MIN_US, SHOULDER_MAX_US);
  elbowServo.attach(ELBOW_PIN, ELBOW_MIN_US, ELBOW_MAX_US);
  wristRotServo.attach(WRIST_ROT_PIN, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US);
  wristBendServo.attach(WRIST_BEND_PIN, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US);

  // Immediately command safe startup pose
  baseServo.writeMicroseconds(degToUs(baseDeg, BASE_MIN_DEG, BASE_MAX_DEG, BASE_MIN_US, BASE_MAX_US));
  shoulderServo.writeMicroseconds(degToUs(shoDeg, SHO_MIN_DEG, SHO_MAX_DEG, SHOULDER_MIN_US, SHOULDER_MAX_US));
  elbowServo.writeMicroseconds(degToUs(elbDeg, ELB_MIN_DEG, ELB_MAX_DEG, ELBOW_MIN_US, ELBOW_MAX_US));
  wristRotServo.writeMicroseconds(degToUs(wristRotDeg, WRIST_ROT_MIN_DEG, WRIST_ROT_MAX_DEG, WRIST_ROT_MIN_US, WRIST_ROT_MAX_US));
  wristBendServo.writeMicroseconds(degToUs(wristBendDeg, WRIST_BEND_MIN_DEG, WRIST_BEND_MAX_DEG, WRIST_BEND_MIN_US, WRIST_BEND_MAX_US));

  // Start in DRIVE mode
  currentMode = MODE_DRIVE;

  Serial.println("Ready. Connect controller.");
}


void loop() {
  BP32.update();
  if (!gp || !gp->isConnected()) return;

  // =====================================================
  // MODE TOGGLE (Circle / BUTTON_B)
  // =====================================================
  bool circle = gp->buttons() & BUTTON_B;

  // Rising edge detection (button JUST pressed)
  if (circle && !prevCircle) {

    // the only timen it is evaluate to true is when the circle is true and the prev is false if(true and !false) --> if (true)

    // Toggle mode
    if (currentMode == MODE_DRIVE) {
      currentMode = MODE_ARM;
      Serial.println("ARM MODE");
    } else {
      currentMode = MODE_DRIVE;
      Serial.println("DRIVE MODE");
    }

    // ---------- Safety actions ONLY on mode change ----------
    stopAllDrive();
    actuatorStop();

    baseStep = 0.0f;
    shoStep  = 0.0f;
    elbStep  = 0.0f;
    wristRotStep  = 0.0f;
    wristBendStep = 0.0f;
  }

  // Save button state for next loop
  prevCircle = circle;

  // =====================================================
  // POWER SOURCE TOGGLE (Triangle / BUTTON_Y)
  // =====================================================
  bool triangle = gp->buttons() & BUTTON_Y;

  if (triangle && !prevTriangle) {
    if (activePower == MAIN_POWER) {
      setSystemPower(POWER_SAFETY);
      Serial.println("SAFETY RELAY ACTIVE");
    } else {
      setSystemPower(MAIN_POWER);
      Serial.println("MAIN RELAY ACTIVE");
    }
  }

  prevTriangle = triangle;

  // =====================================================
  // SUCTION (R1) — TOGGLE
  // =====================================================
  bool R1 = gp->buttons() & BUTTON_SHOULDER_R;

  if (R1 && !prevR1) {
    suctionOn = !suctionOn;

    if (suctionOn) {
      Serial.println("SUCTION ON");
    } else {
      Serial.println("RELEASE");
      digitalWrite(PIN_POMPE, LOW);
      digitalWrite(PIN_VALVE, HIGH);
      delay(100);
      digitalWrite(PIN_VALVE, LOW);
    }
  }

  prevR1 = R1;

  if (suctionOn) {
    digitalWrite(PIN_POMPE, HIGH);
    digitalWrite(PIN_VALVE, LOW);
  }

  // =====================================================
  // MODE DISPATCH
  // =====================================================
  if (currentMode == MODE_DRIVE) {
    handleDriveMode();
  } else {
    handleArmMode();
  }
}
