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

void moveRight(int pwm)
{
    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, pwm); 
    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, pwm);
    analogWrite(AIN1DL, pwm);
    analogWrite(AIN2DL, LOW);
    analogWrite(BIN1DR, pwm);
    analogWrite(BIN2DR, LOW);
}

void moveLeft(int pwm)
{
    analogWrite(AIN1UL, pwm);
    analogWrite(AIN2UL, LOW);
    analogWrite(BIN1UR, pwm);
    analogWrite(BIN2UR, LOW);
    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, pwm);
    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, pwm);
}

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
// Diagonals
void moveForwardRight(int pwm) { // moveForwardRight is working correctly
    analogWrite(AIN1UL, LOW); //clockwise
    analogWrite(AIN2UL, pwm);

    analogWrite(BIN1UR, LOW);  
    analogWrite(BIN2UR, LOW);

    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, LOW);

    analogWrite(BIN1DR, pwm);
    analogWrite(BIN2DR, LOW); 
}

void moveBackwardLeft(int pwm) {
    analogWrite(AIN1UL, pwm);
    analogWrite(AIN2UL, LOW);

    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, LOW);

    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, LOW);

    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, pwm);
}

void moveForwardLeft(int pwm) { // moveForwardLeft is working correctly
    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, LOW);

    analogWrite(BIN1UR, pwm);
    analogWrite(BIN2UR, LOW);

    analogWrite(AIN1DL, LOW);
    analogWrite(AIN2DL, pwm);

    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, LOW);
}

void moveBackwardRight(int pwm) {
    analogWrite(AIN1UL, LOW);
    analogWrite(AIN2UL, LOW);

    analogWrite(BIN1UR, LOW);
    analogWrite(BIN2UR, pwm);

    analogWrite(AIN1DL, pwm);
    analogWrite(AIN2DL, LOW);

    analogWrite(BIN1DR, LOW);
    analogWrite(BIN2DR, LOW);
}

// ======================================================
// GAMEPAD CALLBACK
// ======================================================
void onConnectedGamepad(GamepadPtr gpRef)
{
    gp = gpRef;
    Serial.println("Controller connected !");
}

void onDisconnectedGamepad(GamepadPtr gpRef)
{
    gp = nullptr;
    Serial.println("Controller diconnected!");
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
    Serial.println("Bluepad32 ready. Connect your PS4 controller!");
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

        // DIAGONALS (must be checked before single-axis moves)
        if (joyY < -DEADZONE && joyX > DEADZONE) {
            Serial.println("DIAGONAL: Forward Right");
            moveForwardRight(speedY);
        }
        else if (joyY < -DEADZONE && joyX < -DEADZONE) {
            Serial.println("DIAGONAL: Forward Left");
            moveForwardLeft(speedY);
        }
        else if (joyY > DEADZONE && joyX > DEADZONE) {
            Serial.println("DIAGONAL: Backward Right");
            moveBackwardRight(speedY);
        }
        else if (joyY > DEADZONE && joyX < -DEADZONE) {
            Serial.println("DIAGONAL: Backward Left");
            moveBackwardLeft(speedY);
        }

        else if (joyRX > DEADZONE) {
            spinRight(speedRX);
        }
        else if (joyRX > DEADZONE)
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
