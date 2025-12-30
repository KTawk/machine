#include <Arduino.h>
#include <Bluepad32.h>

// =======================
// L298N MOTOR PINS (Motor A)
// =======================

#define IN1_PIN 23
#define IN2_PIN 26

GamepadPtr gp = nullptr;

// =======================
// GAMEPAD CALLBACKS
// =======================
void onConnectedGamepad(GamepadPtr gpRef) {
    gp = gpRef;
    Serial.println("Stadia controller connected");
}

void onDisconnectedGamepad(GamepadPtr gpRef) {
    gp = nullptr;

    // Sécurité : moteur OFF
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);

    Serial.println("Controller disconnected");
}

// =======================
// SETUP
// =======================
void setup() {
    Serial.begin(115200);


    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);

    // Sécurité au démarrage
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);

    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    Serial.println("Ready. Connect Stadia controller.");
}

// =======================
// LOOP
// =======================
void loop() {
    BP32.update();

    if (!gp || !gp->isConnected()) {
        return;
    }

    // Lecture correcte du D-PAD (Stadia)
    uint8_t dpad = gp->dpad();
    bool up   = (dpad == DPAD_UP);
    bool down = (dpad == DPAD_DOWN);

    if (up && !down) {
        // Sens 1
        digitalWrite(IN1_PIN, HIGH);
        digitalWrite(IN2_PIN, LOW);

        Serial.println("DPAD UP → moteur sens 1");

    } else if (down && !up) {
        // Sens 2
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, HIGH);

        Serial.println("DPAD DOWN → moteur sens 2");

    } else {
        // STOP
        digitalWrite(IN1_PIN, LOW);
        digitalWrite(IN2_PIN, LOW);

    }

    delay(10); // stabilité
}
﻿
