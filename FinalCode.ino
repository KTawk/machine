#include <Bluepad32.h>
#include <ESP32Servo.h>

// ================== PINS ==================
#define SERVOR_PIN    27
#define SERVOL_PIN    26
#define GRIPPER_PIN   25

// ================== GRIPPER ==================
#define CLOSED_ANGLE   60
#define OPEN_ANGLE    120

// ================== WHEELS ==================
#define NEUTRAL_US   1530
#define MAX_US       2000
#define MIN_US       1000
#define DEADZONE     40
#define INVERT_STICK true

// ================== GLOBALS ==================
Servo servoR;
Servo servoL;
Servo gripperServo;

// UNE SEULE MANETTE ACTIVE (comme le 1er code)
ControllerPtr activeController = nullptr;

// Gripper
bool gripperOpen = false;
bool prevR1 = false;

// ================== UTILS ==================
static inline bool isNeutral(int v) {
  return abs(v) <= DEADZONE;
}

// ================== CALLBACKS (IDENTIQUES AU 1er CODE) ==================
void onConnectedController(ControllerPtr ctl) {

  if (activeController == nullptr) {
    activeController = ctl;

    // 🔒 Verrou Bluetooth : FIRST CONTROLLER WINS
    BP32.enableNewBluetoothConnections(false);

    Serial.println("Manette ACCEPTÉE (première)");
  }
  else {
    Serial.println("Manette REFUSÉE (déjà une active)");
  }
}

void onDisconnectedController(ControllerPtr ctl) {

  if (ctl == activeController) {
    activeController = nullptr;

    // 🛑 Sécurité identique au 1er code
    servoR.writeMicroseconds(NEUTRAL_US);
    servoL.writeMicroseconds(NEUTRAL_US);
    gripperServo.write(CLOSED_ANGLE);
    gripperOpen = false;

    // 🔓 Réouvrir Bluetooth pour reconnexion
    BP32.enableNewBluetoothConnections(true);

    Serial.println("Manette PERDUE - attente reconnexion");
  }
}

// ================== GAMEPAD PROCESS ==================
void processGamepad(ControllerPtr ctl) {

  // ---------- R1 TOGGLE (GRIPPER) ----------
  bool r1 = (ctl->buttons() & BUTTON_SHOULDER_R);

  if (r1 && !prevR1) {
    gripperOpen = !gripperOpen;
    gripperServo.write(gripperOpen ? OPEN_ANGLE : CLOSED_ANGLE);
  }
  prevR1 = r1;

  // ---------- STICKS (ROUES) ----------
  int stickL = ctl->axisY();
  int stickR = ctl->axisRY();

  if (INVERT_STICK) {
    stickL = -stickL;
    stickR = -stickR;
  }

  bool L_neutral = isNeutral(stickL);
  bool R_neutral = isNeutral(stickR);

  int pulseL = NEUTRAL_US;
  int pulseR = NEUTRAL_US;

  if (!L_neutral && !R_neutral) {
    if (stickL > DEADZONE && stickR > DEADZONE) {
      pulseL = MAX_US; pulseR = MIN_US;
    } else if (stickL < -DEADZONE && stickR < -DEADZONE) {
      pulseL = MIN_US; pulseR = MAX_US;
    } else if (stickL > DEADZONE && stickR < -DEADZONE) {
      pulseL = MAX_US; pulseR = MAX_US;
    } else if (stickL < -DEADZONE && stickR > DEADZONE) {
      pulseL = MIN_US; pulseR = MIN_US;
    }
  }
  else if (!L_neutral) {
    pulseL = (stickL > 0) ? MAX_US : MIN_US;
  }
  else if (!R_neutral) {
    pulseR = (stickR > 0) ? MIN_US : MAX_US;
  }

  servoR.writeMicroseconds(pulseR);
  servoL.writeMicroseconds(pulseL);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("=== BOOT ROBOT ===");
  Serial.println("Bluetooth CLEAN + FIRST CONTROLLER WINS");

  // Callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // 🔥 CLEAN AVANT TOUTE CONNEXION (comme le 1er code)
  BP32.forgetBluetoothKeys();

  // Bluetooth ouvert au départ
  BP32.enableNewBluetoothConnections(true);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoR.setPeriodHertz(50);
  servoL.setPeriodHertz(50);

  servoR.attach(SERVOR_PIN, 1000, 2000);
  servoL.attach(SERVOL_PIN, 1000, 2000);

  servoR.writeMicroseconds(NEUTRAL_US);
  servoL.writeMicroseconds(NEUTRAL_US);

  gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(CLOSED_ANGLE);
}

// ================== LOOP ==================
void loop() {
  BP32.update();

  if (activeController &&
      activeController->isConnected() &&
      activeController->hasData()) {

    processGamepad(activeController);
  }
  else {
    // Sécurité si aucune manette
    servoR.writeMicroseconds(NEUTRAL_US);
    servoL.writeMicroseconds(NEUTRAL_US);
  }

  delay(20);
}

