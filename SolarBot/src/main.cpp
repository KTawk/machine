#include <Arduino.h>
#include <Bluepad32.h>

// === Pin Definitions for Motor Driver ===
#define STBY 23

#define AIN1UL 21
#define AIN2UL 22
#define BIN1UR 19
#define BIN2UR 18
#define AIN1DL 4
#define AIN2DL 5
#define BIN1DR 17
#define BIN2DR 16

#define MAX_SPEED 255
#define DEADZONE 80

GamepadPtr gp = nullptr;

// ======================================================
// MOTEUR CONTROL
// ======================================================
void stopMotors()
{
    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, LOW);
    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, LOW);
    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, LOW);
    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, LOW);
}

// Forward
void moveForward(int pwm)
{
    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, pwm);
    analogWrite(BIN1UR, pwm);
    analogWrite(BIN2UR, LOW);
    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, pwm);
    analogWrite(BIN1DR, pwm);
    analogWrite(BIN2DR, LOW);
}

// Backward
void moveBackward(int pwm)
{
    analogWrite(AIN1UL, pwm);
    analogWrite(AIN2UL, LOW);
    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, pwm);
    analogWrite(AIN1DL, pwm);
    analogWrite(AIN2DL, LOW);
    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, pwm);
}

// Right
void moveRight(int pwm)
{
    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, pwm); //ok
    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, pwm);//ok
    analogWrite(AIN1DL, pwm);
    analogWrite(AIN2DL, LOW);
    analogWrite(BIN1DR, pwm);
    analogWrite(BIN2DR, LOW);
}

// Left
void moveLeft(int pwm)
{
    analogWrite(AIN1UL, pwm);
    analogWrite(AIN2UL, LOW);//ok
    analogWrite(BIN1UR, pwm);
    analogWrite(BIN2UR, LOW);//ok
    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, pwm);
    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, pwm);
}

// SPIN (rotation sur place)
void spinRight(int pwm)
{
    Serial.println("SPIN RIGHT");

    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, pwm);

    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, pwm);

    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, pwm);

    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, pwm);
}

void spinLeft(int pwm)
{
    Serial.println("SPIN LEFT");

    analogWrite(AIN1UL, pwm);
    analogWrite(AIN2UL, LOW);

    analogWrite(BIN1UR, pwm);
    analogWrite(BIN2UR, LOW);

    analogWrite(AIN1DL, pwm);
    analogWrite(AIN2DL, LOW);

    analogWrite(BIN1DR, pwm);
    analogWrite(BIN2DR, LOW);
}
// ======================================================
// GAMEPAD CALLBACK
// ======================================================
void onConnectedGamepad(GamepadPtr gpRef)
{
    gp = gpRef;
    Serial.println("Manette connectée !");
}

void onDisconnectedGamepad(GamepadPtr gpRef)
{
    gp = nullptr;
    Serial.println("Manette déconnectée !");
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);

    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);

    pinMode(AIN1UL, OUTPUT);
    pinMode(AIN2UL, OUTPUT);
    pinMode(BIN1UR, OUTPUT);
    pinMode(BIN2UR, OUTPUT);
    pinMode(AIN1DL, OUTPUT);
    pinMode(AIN2DL, OUTPUT);
    pinMode(BIN1DR, OUTPUT);
    pinMode(BIN2DR, OUTPUT);

    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();
    Serial.println("Bluepad32 prêt. Connecte ta manette PS4 !");
}

// ======================================================
// LOOP
// ======================================================
void loop()
{
    BP32.update();

    if (gp && gp->isConnected())
    {

        int joyX = gp->axisX();   // joystick gauche X
        int joyY = gp->axisY();   // joystick gauche Y
        int joyRX = gp->axisRX(); // joystick droit X (pour spin)

        // Deadzones
        if (abs(joyX) < DEADZONE)
            joyX = 0;
        if (abs(joyY) < DEADZONE)
            joyY = 0;
        if (abs(joyRX) < DEADZONE)
            joyRX = 0;

        // Mapping to speed
        int speedX = map(abs(joyX), 0, 512, 0, MAX_SPEED);
        int speedY = map(abs(joyY), 0, 512, 0, MAX_SPEED);
        int speedRX = map(abs(joyRX), 0, 512, 0, MAX_SPEED);

        // PRIORITÉ : SPIN > MOUVEMENT
        if (joyRX > DEADZONE)
        {
            spinRight(speedRX);
        }
        else if (joyY < -DEADZONE)
        {
            Serial.println("FORWARD");
            moveForward(speedY);
        }
        else if (joyY > DEADZONE)
        {
            Serial.println("BACKWARD");
            moveBackward(speedY);
        }
        else if (joyX > DEADZONE)
        {
            Serial.println("RIGHT");
            moveRight(speedX);
        }
        else if (joyX < -DEADZONE)
        {
            Serial.println("LEFT");
            moveLeft(speedX);
        }
        else if (joyRX < -DEADZONE)
        {
            spinLeft(speedRX);
        }
        else
        {
            stopMotors();
            Serial.println("STOP");
        }
    }

    delay(20);
}
