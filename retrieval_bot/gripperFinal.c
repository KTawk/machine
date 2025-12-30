#include <Bluepad32.h>
#include <ESP32Servo.h>

#define SERVO_PIN      18    // Servo control pin
#define CLOSED_ANGLE    0    // Closed position
#define OPEN_ANGLE     60    // Open position

Servo gripperServo;
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

bool gripperOpen = false;
int currentAngle = CLOSED_ANGLE;

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Controller connected (index=%d)\n", i);
      return;
    }
  }
  Serial.println("⚠️ No empty slot for new controller.");
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
  bool r1 = ctl->buttons() & BUTTON_SHOULDER_R;  // PS4 R1
  bool l1 = ctl->buttons() & BUTTON_SHOULDER_L;  // PS4 L1

  if (r1 && !gripperOpen) {
    gripperServo.write(OPEN_ANGLE);
    gripperOpen = true;
    currentAngle = OPEN_ANGLE;
    Serial.println("Gripper opened!");
  }

  if (l1 && gripperOpen) {
    gripperServo.write(CLOSED_ANGLE);
    gripperOpen = false;
    currentAngle = CLOSED_ANGLE;
    Serial.println("Gripper closed!");
  }
}


void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      processGamepad(ctl);
      return;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("ESP32 BT Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  gripperServo.attach(SERVO_PIN);
  gripperServo.write(CLOSED_ANGLE); // Start closed

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); // optional
}

void loop() {
  if (BP32.update()) {
    processControllers();
  }
  delay(20);
}
