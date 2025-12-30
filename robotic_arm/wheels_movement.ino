#include <Arduino.h> 
#include <Bluepad32.h>


#define STBY 23

#define UL1 5
#define UL2 17
#define UR1 19
#define UR2 18
#define DL1 22
#define DL2 21
#define DR1 15
#define DR2 2


#define DEADZONE 40
#define MAX_PWM 255

GamepadPtr gp = nullptr;

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
// OMNI DRIVE + SPIN (adapté à TON châssis)
// ======================================================
void omniDrive(int joyX, int joyY, int joyRX) {

    // Deadzones
    if (abs(joyX) < DEADZONE) joyX = 0;
    if (abs(joyY) < DEADZONE) joyY = 0;
    if (abs(joyRX) < DEADZONE) joyRX = 0;

    if (joyX == 0 && joyY == 0 && joyRX == 0) {
        stopAll();
        return;
    }

    // ---- Translation (tes axes calibrés) ----
    float X = -joyX / 509.0;
    float Y =  joyY / 509.0;

    float angle = atan2(Y, X);

    float R = sqrt(X*X + Y*Y);
    float Speed = R * R * 255;

    float ul = cos(angle - PI/4)   * Speed;
    float ur = cos(angle + PI/4)   * Speed;
    float dl = cos(angle - 3*PI/4) * Speed;
    float dr = cos(angle + 3*PI/4) * Speed;

    // -------- ROTATION FLIPPED (RIGHT STICK X) --------
    const float SPIN_GAIN = 0.8;
    float spin = -(joyRX / 509.0) * 255 * SPIN_GAIN;  
    // ---------------------------------------------------

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
    Serial.println("PS4 Controller connected!");
}

void onDisconnectedGamepad(GamepadPtr gpRef) {
    gp = nullptr;
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

    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    Serial.println("Ready. Connect PS4 controller.");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    BP32.update();

    if (gp && gp->isConnected()) {
        int joyX  = gp->axisX();   // Left stick X
        int joyY  = gp->axisY();   // Left stick Y
        int joyRX = gp->axisRX();  // Right stick X (spin)

        omniDrive(joyX, joyY, joyRX);
    }

    delay(15);
}
