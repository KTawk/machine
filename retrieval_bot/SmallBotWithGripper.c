#include <Bluepad32.h>
#include <ESP32Servo.h>

#define SERVOR_PIN    27    
#define SERVOL_PIN    26    
#define GRIPPER_PIN   25    

// Gripper angles
#define CLOSED_ANGLE   0
#define OPEN_ANGLE    60

// Wheel pulse definitions (microseconds)
#define NEUTRAL_US   1530   
#define MAX_US       2000   
#define MIN_US       1000   
#define DEADZONE     15     
#define INVERT_STICK true   

// Globals
Servo servoR;   
Servo servoL;   
Servo gripperServo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

bool gripperOpen = false;
int currentAngle = CLOSED_ANGLE;

static inline bool isNeutral(int v) { return abs(v) <= DEADZONE; }

// ---------- Bluepad32 callbacks ----------
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

// ---------- Process single controller: wheels + gripper ----------
void processGamepad(ControllerPtr ctl) {
  // --- Buttons for gripper ---
  bool r1 = (ctl->buttons() & BUTTON_SHOULDER_R);  // R1
  bool l1 = (ctl->buttons() & BUTTON_SHOULDER_L);  // L1

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

  // --- Sticks for wheels ---
  // Bluepad32 axis functions return values approximately in the range [-128..+127]
  int stickL = ctl->axisY();   
  int stickR = ctl->axisRY();  

  if (INVERT_STICK) {
    // If controller yields UP as negative, negate to make UP positive
    stickL = -stickL;
    stickR = -stickR;
  }

  const bool L_neutral = isNeutral(stickL);
  const bool R_neutral = isNeutral(stickR);

  int pulseL = NEUTRAL_US;
  int pulseR = NEUTRAL_US;

  // Priority 1: both sticks used -> coordinated movement
  if (!L_neutral && !R_neutral) {
    if (stickL > DEADZONE && stickR > DEADZONE) {
      // both UP -> forward 
      pulseL = MAX_US;
      pulseR = MIN_US;
    } else if (stickL < -DEADZONE && stickR < -DEADZONE) {
      // both DOWN -> backward
      pulseL = MIN_US;
      pulseR = MAX_US;
    } else if (stickL > DEADZONE && stickR < -DEADZONE) {
      // L up, R down -> spin one way
      pulseL = MAX_US;
      pulseR = MAX_US;
    } else if (stickL < -DEADZONE && stickR > DEADZONE) {
      // L down, R up -> spin other way
      pulseL = MIN_US;
      pulseR = MIN_US;
    } else {
      // conflicting or small values -> neutral
      pulseL = NEUTRAL_US;
      pulseR = NEUTRAL_US;
    }
  }
  // Priority 2: single stick drives its side
  else if (!L_neutral && R_neutral) {
    pulseL = (stickL > 0) ? MAX_US : MIN_US;
    pulseR = NEUTRAL_US;
  }
  else if (!R_neutral && L_neutral) {
    pulseR = (stickR > 0) ? MIN_US : MAX_US;
    pulseL = NEUTRAL_US;
  }
  // Priority 3: both neutral -> stop
  else {
    pulseL = NEUTRAL_US;
    pulseR = NEUTRAL_US;
  }

  // Write the pulses
  servoR.writeMicroseconds(pulseR);
  servoL.writeMicroseconds(pulseL);

  // Debug print
  Serial.printf("L:%4d -> %4dus  |  R:%4d -> %4dus\n", stickL, pulseL, stickR, pulseR);
}

void processControllers() {
  for (auto ctl : myControllers) {
    if (ctl && ctl->isConnected() && ctl->hasData()) {
      processGamepad(ctl);
      return; 
    }
  }
}

// ---------- Setup & Loop ----------
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println();

  // Print firmware and BT addr (optional but useful)
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("ESP32 BT Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); // optional: remove previously paired keys

  // ESP32Servo / PWM timers (required before attaching multiple servos)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Wheel servos - use microseconds attach for safe range
  servoR.setPeriodHertz(50);
  servoL.setPeriodHertz(50);
  servoR.attach(SERVOR_PIN, 1000, 2000);
  servoL.attach(SERVOL_PIN, 1000, 2000);

  servoR.writeMicroseconds(NEUTRAL_US);
  servoL.writeMicroseconds(NEUTRAL_US);

  // Gripper servo - angles
  gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(CLOSED_ANGLE); 

  Serial.println("Setup complete. Waiting for controller...");
}

void loop() {
  if (BP32.update()) {
    processControllers();
  } else {
    servoR.writeMicroseconds(NEUTRAL_US);
    servoL.writeMicroseconds(NEUTRAL_US);
  }
  delay(20);
}
