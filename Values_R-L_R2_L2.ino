#include <Bluepad32.h>
#include <Arduino.h>
#include <ESP32Servo.h>
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define SERVO_PIN 18
#define MIN_ANGLE 5            // safer mechanical limits (adjust if tested)
#define MAX_ANGLE 175
#define TRIGGER_MAX_DEG_PER_S 100   // raise/lower to taste
#define DEADZONE 12        


// State
Servo gripper;
float angleDeg = (MIN_ANGLE + MAX_ANGLE) * 0.5f;
uint32_t lastMs = 0;
         // ignore tiny trigger noise (0..255)

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("PS4 Controller connected (index=%d)\n", i);
      return;
    }
  }
  Serial.println("No empty slot for new controller.");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      Serial.printf("Controller disconnected (index=%d)\n", i);
      return;
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  int lx = ctl->axisX();   // Left stick X (-511 to 512)
  int ly = ctl->axisY();   // Left stick Y (-511 to 512)
  int rx = ctl->axisRX();  // Right stick X (-511 to 512)
  int ry = ctl->axisRY();  // Right stick Y (-511 to 512)
  int l2 = ctl->brake();   // L2 trigger (0 to 1023)
  int r2 = ctl->throttle(); // R2 trigger (0 to 1023)

  Serial.printf("LX:%4d  LY:%4d  RX:%4d  RY:%4d  L2:%4d  R2:%4d\n", lx, ly, rx, ry, l2, r2);
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      processGamepad(ctl);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("ESP32 BT Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // optional: clear old pairings

  // ESP32Servo best-practice setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gripper.setPeriodHertz(50);                 // 50 Hz for servos
  gripper.attach(SERVO_PIN, 500, 2500);       // widen if your servo supports it

  gripper.write((int)round(angleDeg));
  lastMs = millis();
}

void loop() {
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  // Cap dt to avoid big jumps after stalls
  if (dt > 0.05f) dt = 0.05f; // treat anything >50 ms as 50 ms
  if (BP32.update())
    processControllers();

  delay(100);

  // Find connected controller
  ControllerPtr ctl = nullptr;
  for (auto c : myControllers) {
    if (c && c->isConnected()) {
      ctl = c;
      break;
    }
  }

  if (!ctl)
    return; // no controller connected

  // Proportional trigger control with deadzone
  int r2 = ctl->throttle(); // open
  int l2 = ctl->brake();    // close

  if (r2 > DEADZONE || l2 > DEADZONE) {
    float openSpd  = map(r2, DEADZONE, 1020, 0, TRIGGER_MAX_DEG_PER_S);
    float closeSpd = map(l2, DEADZONE, 1020, 0, TRIGGER_MAX_DEG_PER_S);
    if (openSpd < 0) openSpd = 0;
    if (closeSpd < 0) closeSpd = 0;

    float net = openSpd - closeSpd;
    angleDeg = constrain(angleDeg + net * dt, MIN_ANGLE, MAX_ANGLE);
  }

  static float lastCmd = -999;
  if (fabs(angleDeg - lastCmd) >= 0.5f) {
    gripper.write((int)round(angleDeg));
    lastCmd = angleDeg;
  }
  delay(2);
}

