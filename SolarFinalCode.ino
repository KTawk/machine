#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ======================================================
// ---------------- PINOUT -------------------------------
// ======================================================
#define STBY 5

// ---- Motors (Mecanum) ----
#define UL1 22
#define UL2 21
#define UR1 18
#define UR2 27
#define DL1 32
#define DL2 33
#define DR1 26
#define DR2 25

// ---- Vacuum Pump --------------------------------------
#define PIN_POMPE 4

// ---- Servo --------------------------------------------
#define SERVO_PIN 19
Servo gripperServo;

// ======================================================
#define DEADZONE 40
#define MAX_PWM 255

// ======================================================
// GAMEPAD (UNE seule manette)
// ======================================================
GamepadPtr activeGp = nullptr;

// ======================================================
// --------- STATES -------------------------------------
// ======================================================
bool driveFlipped = false;
bool prevCircle = false;

bool slowMode = false;
bool prevX = false;
float speedScale = 1.0;

bool suctionOn = false;
bool prevR1 = false;
uint8_t r1PressCount = 0;

bool servoOpen = false;
bool prevL1 = false;
int servoAngle = 0;

// ======================================================
// MOTOR CONTROL
// ======================================================
void setMotor(int in1, int in2, float pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, -pwm);
  }
}

void stopAll() {
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
    stopAll();
    return;
  }

  float X = -joyX / 509.0;
  float Y =  joyY / 509.0;

  float angle = atan2(Y, X);
  float R = sqrt(X * X + Y * Y);

  float Speed = R * R * 255 * speedScale;

  float ul, ur, dl, dr;

  if (!driveFlipped) {
    ul = cos(angle - PI/4) * Speed;
    ur = cos(angle + PI/4) * Speed;
    dl = cos(angle - 3*PI/4) * Speed;
    dr = cos(angle + 3*PI/4) * Speed;
  } else {
    ul = cos(angle - PI/4 + PI) * Speed;
    ur = cos(angle + PI/4 + PI) * Speed;
    dl = cos(angle - 3*PI/4 + PI) * Speed;
    dr = cos(angle + 3*PI/4 + PI) * Speed;
  }

  float spin = -(joyRX / 509.0) * abs(joyRX / 509.0) * 255 * 0.5 * speedScale;

  setMotor(UL1, UL2, ul + spin);
  setMotor(UR1, UR2, ur + spin);
  setMotor(DL1, DL2, dl + spin);
  setMotor(DR1, DR2, dr + spin);
}

// ======================================================
// GAMEPAD CALLBACKS (LOGIQUE CLEAN + LOCK)
// ======================================================
void onConnectedGamepad(GamepadPtr gpRef) {

  if (activeGp == nullptr) {
    activeGp = gpRef;

    // 🔒 Verrou Bluetooth
    BP32.enableNewBluetoothConnections(false);

    Serial.println("Manette ACCEPTÉE (première)");
  } else {
    Serial.println("Manette REFUSÉE (déjà une active)");
  }
}

void onDisconnectedGamepad(GamepadPtr gpRef) {

  if (gpRef == activeGp) {
    activeGp = nullptr;

    stopAll();
    digitalWrite(PIN_POMPE, LOW);
    gripperServo.write(0);

    // 🔓 Réouvrir Bluetooth pour reconnexion
    BP32.enableNewBluetoothConnections(true);

    Serial.println("Manette PERDUE - attente reconnexion");
  }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("=== BOOT ROBOT ===");
  Serial.println("Bluetooth CLEAN + FIRST CONTROLLER WINS");

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(UL1, OUTPUT); pinMode(UL2, OUTPUT);
  pinMode(UR1, OUTPUT); pinMode(UR2, OUTPUT);
  pinMode(DL1, OUTPUT); pinMode(DL2, OUTPUT);
  pinMode(DR1, OUTPUT); pinMode(DR2, OUTPUT);

  pinMode(PIN_POMPE, OUTPUT);
  digitalWrite(PIN_POMPE, LOW);

  gripperServo.attach(SERVO_PIN);
  gripperServo.write(0);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // 🔥 CLEAN AVANT TOUTE CONNEXION
  BP32.forgetBluetoothKeys();

  // Bluetooth ouvert au départ
  BP32.enableNewBluetoothConnections(true);

  Serial.println("Attente de la PREMIÈRE manette...");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  BP32.update();

  if (!activeGp || !activeGp->isConnected()) return;

  uint16_t buttons = activeGp->buttons();

  bool circle = buttons & BUTTON_B;
  if (circle && !prevCircle) driveFlipped = !driveFlipped;
  prevCircle = circle;

  bool Xbtn = buttons & BUTTON_X;
  if (Xbtn && !prevX) {
    slowMode = !slowMode;
    speedScale = slowMode ? 0.3 : 1.0;
  }
  prevX = Xbtn;

  omniDrive(
    activeGp->axisX(),
    activeGp->axisY(),
    activeGp->axisRX()
  );

  bool R1 = buttons & BUTTON_SHOULDER_R;
  if (R1 && !prevR1) {
    suctionOn = !suctionOn;
  }
  prevR1 = R1;
  digitalWrite(PIN_POMPE, suctionOn ? HIGH : LOW);

  bool L1 = buttons & BUTTON_SHOULDER_L;
  if (L1 && !prevL1) {
    servoOpen = !servoOpen;
    servoAngle = servoOpen ? 180 : 0;
  }
  prevL1 = L1;

  int R2 = activeGp->throttle();
  int L2 = activeGp->brake();

  float step = max(1.0, 4.0 * speedScale);
  if (R2 > 50) servoAngle += step;
  if (L2 > 50) servoAngle -= step;

  servoAngle = constrain(servoAngle, 0, 180);
  gripperServo.write(servoAngle);

  delay(15);
}
