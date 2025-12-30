#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ======================================================
// ---------------- PINOUT -------------------------------
// ======================================================
#define STBY 5        // STBY commun (moteurs + pompe)

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

GamepadPtr gp = nullptr;

// ============================================================
// --------- DRIVE TOGGLE (Circle) ----------------------------
// ============================================================
bool driveFlipped = false;
bool prevCircle = false;

// ============================================================
// --------- SPEED PRECISION MODE (X button) ------------------
// ============================================================
bool slowMode = false;
bool prevX = false;
float speedScale = 1.0;   // 1.0 = normal, 0.5 = précision

// =============================================================
// --------- SUCTION STATE MACHINE -----------------------------
// =============================================================
bool suctionOn = false;
bool prevR1 = false;
uint8_t r1PressCount = 0;

// ============================================================
// --------- SERVO STATE MACHINE ------------------------------
// ============================================================
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
// OMNI DRIVE + SPIN (MECANUM)
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

    // ---------- SPEED (avec mode précision) ----------
    float Speed = R * R * 255 * speedScale;

    float ul, ur, dl, dr;

    if (!driveFlipped) {
        ul = cos(angle - PI/4)        * Speed;
        ur = cos(angle + PI/4)        * Speed;
        dl = cos(angle - 3*PI/4)      * Speed;
        dr = cos(angle + 3*PI/4)      * Speed;
    } else {
        ul = cos(angle - PI/4 + PI)   * Speed;
        ur = cos(angle + PI/4 + PI)   * Speed;
        dl = cos(angle - 3*PI/4 + PI) * Speed;
        dr = cos(angle + 3*PI/4 + PI) * Speed;
    }

    // ---------- SPIN (avec mode précision) ----------
    const float SPIN_GAIN = 0.5;
    float spin = -(joyRX / 509.0) * abs(joyRX / 509.0) * 255 * SPIN_GAIN * speedScale;

    ul += spin;
    ur += spin;
    dl += spin;
    dr += spin;

    setMotor(UL1, UL2, ul);
    setMotor(UR1, UR2, ur);
    setMotor(DL1, DL2, dl);
    setMotor(DR1, DR2, dr);
}

// ======================================================
// GAMEPAD CALLBACKS
// ======================================================
void onConnectedGamepad(GamepadPtr gpRef) {
    gp = gpRef;
    Serial.println("Stadia controller connected!");
}

void onDisconnectedGamepad(GamepadPtr gpRef) {
    gp = nullptr;
    stopAll();
    digitalWrite(PIN_POMPE, LOW);
    gripperServo.write(0);
    Serial.println("Controller disconnected!");
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
    digitalWrite(PIN_POMPE, LOW);

    gripperServo.attach(SERVO_PIN);
    gripperServo.write(0);
    servoAngle = 0;

    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    Serial.println("Ready. Connect Stadia controller.");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    BP32.update();

    if (!gp || !gp->isConnected()) return;

    uint16_t buttons = gp->buttons();

    // ----------- DRIVE MODE TOGGLE (Circle) ------------
    bool circle = buttons & BUTTON_B;
    if (circle && !prevCircle) {
        driveFlipped = !driveFlipped;
        Serial.println(driveFlipped ? "Drive Mode B" : "Drive Mode A");
    }
    prevCircle = circle;

    // ----------- SPEED PRECISION TOGGLE (X) ------------
    bool Xbtn = buttons & BUTTON_X;
    if (Xbtn && !prevX) {
        slowMode = !slowMode;
        speedScale = slowMode ? 0.3 : 1.0;
        Serial.println(slowMode ? "Precision mode ON (50%)" : "Normal speed");
    }
    prevX = Xbtn;

    // ----------- MOVEMENT ------------------------------
    omniDrive(
        gp->axisX(),
        gp->axisY(),
        gp->axisRX()
    );

    // ----------- SUCTION (R1 toggle) -------------------
    bool R1 = buttons & BUTTON_SHOULDER_R;
    if (R1 && !prevR1) {
        r1PressCount++;
        if (r1PressCount > 2) r1PressCount = 1;
        suctionOn = (r1PressCount == 1);
    }
    prevR1 = R1;
    digitalWrite(PIN_POMPE, suctionOn ? HIGH : LOW);

    // ----------- SERVO TOGGLE (L1) ---------------------
    bool L1 = buttons & BUTTON_SHOULDER_L;
    if (L1 && !prevL1) {
        servoOpen = !servoOpen;
        servoAngle = servoOpen ? 180 : 0;
        gripperServo.write(servoAngle);
    }
    prevL1 = L1;

    // ---- Servo continu (R2 / L2)
    int R2 = gp->throttle(); // R2
    int L2 = gp->brake();    // L2

   // ---- Servo speed scaling (comme les roues)
   float servoStep = 4.0 * speedScale;

   // Sécurité : éviter que ça tombe à 0
   if (servoStep < 1.0) servoStep = 1.0;

   if (R2 > 50) servoAngle += servoStep;
   if (L2 > 50) servoAngle -= servoStep;


    servoAngle = constrain(servoAngle, 0, 180);
    gripperServo.write(servoAngle);

    delay(15);
}
