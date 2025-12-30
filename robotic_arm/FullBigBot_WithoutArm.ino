#include <Arduino.h>
#include <Bluepad32.h>

// ======================================================
// PINS – SUCTION (pompe + valve)
// ======================================================
#define PIN_POMPE 32
#define PIN_VALVE 25

// ======================================================
// PINS – MECANUM / OMNI DRIVE
// ======================================================
#define STBY 33

#define UL1 5
#define UL2 17
#define UR1 19
#define UR2 18
#define DL1 22
#define DL2 21
#define DR1 15
#define DR2 2

// ======================================================
// PINS – LINEAR ACTUATOR (L298N / pont en H)
// ======================================================
#define ACT_IN1 23
#define ACT_IN2 26

// ======================================================
// PARAMS
// ======================================================
#define DEADZONE 40
#define MAX_PWM  255

// ======================================================
// GLOBALS
// ======================================================
GamepadPtr gp = nullptr;

// Suction
bool suctionOn = false;
bool prevR1 = false;
uint8_t r1PressCount = 0;

// ======================================================
// MOTOR CONTROL (mecanum)
// ======================================================
void setMotor(int in1, int in2, float pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    analogWrite(in1, (int)pwm);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, (int)(-pwm));
  }
}

void stopAllDrive() {
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
    stopAllDrive();
    return;
  }

  float X = -joyX / 509.0f;
  float Y =  joyY / 509.0f;

  float angle = atan2f(Y, X);
  float R = sqrtf(X * X + Y * Y);
  float speed = R * R * 255.0f;

  float ul = cosf(angle - PI / 4)      * speed;
  float ur = cosf(angle + PI / 4)      * speed;
  float dl = cosf(angle - 3 * PI / 4)  * speed;
  float dr = cosf(angle + 3 * PI / 4)  * speed;

  const float SPIN_GAIN = 0.8f;
  float spin = -(joyRX / 509.0f) * 255.0f * SPIN_GAIN;

  ul += spin;
  ur -= spin;
  dl += spin;
  dr -= spin;

  float maxv = max(max(fabs(ul), fabs(ur)), max(fabs(dl), fabs(dr)));
  if (maxv > 255.0f) {
    float k = 255.0f / maxv;
    ul *= k; ur *= k; dl *= k; dr *= k;
  }

  setMotor(UL1, UL2, ul);
  setMotor(UR1, UR2, ur);
  setMotor(DL1, DL2, dl);
  setMotor(DR1, DR2, dr);
}

// ======================================================
// ACTUATOR CONTROL (DPAD UP / DOWN)
// ======================================================
void actuatorStop() {
  digitalWrite(ACT_IN1, LOW);
  digitalWrite(ACT_IN2, LOW);
}

void actuatorForward() {
  digitalWrite(ACT_IN1, HIGH);
  digitalWrite(ACT_IN2, LOW);
}

void actuatorReverse() {
  digitalWrite(ACT_IN1, LOW);
  digitalWrite(ACT_IN2, HIGH);
}

// ======================================================
// GAMEPAD CALLBACKS
// ======================================================
void onConnectedGamepad(GamepadPtr gpRef) {
  gp = gpRef;
  Serial.println("Controller connected!");
}

void onDisconnectedGamepad(GamepadPtr gpRef) {
  (void)gpRef;
  gp = nullptr;

  stopAllDrive();
  actuatorStop();

  suctionOn = false;
  digitalWrite(PIN_POMPE, LOW);
  digitalWrite(PIN_VALVE, LOW);

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
  pinMode(PIN_VALVE, OUTPUT);
  digitalWrite(PIN_POMPE, LOW);
  digitalWrite(PIN_VALVE, LOW);

  pinMode(ACT_IN1, OUTPUT);
  pinMode(ACT_IN2, OUTPUT);
  actuatorStop();

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();

  Serial.println("Ready. Connect controller.");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  BP32.update();
  if (!gp || !gp->isConnected()) return;

  // ---- DRIVE ----
  omniDrive(gp->axisX(), gp->axisY(), gp->axisRX());

  // ---- ACTUATOR (DPAD) ----
  uint8_t dpad = gp->dpad();
  if (dpad == DPAD_UP) {
    actuatorForward();
  } else if (dpad == DPAD_DOWN) {
    actuatorReverse();
  } else {
    actuatorStop();
  }

  // ---- SUCTION (R1 TOGGLE) ----
  bool R1 = gp->buttons() & BUTTON_SHOULDER_R;

  if (R1 && !prevR1) {
    r1PressCount = (r1PressCount % 2) + 1;

    if (r1PressCount == 1) {
      suctionOn = true;
      Serial.println("SUCTION ON");
    } else {
      suctionOn = false;
      Serial.println("RELEASE");
      digitalWrite(PIN_POMPE, LOW);
      digitalWrite(PIN_VALVE, HIGH);
      delay(100);
      digitalWrite(PIN_VALVE, LOW);
    }
  }
  prevR1 = R1;

  if (suctionOn) {
    digitalWrite(PIN_POMPE, HIGH);
    digitalWrite(PIN_VALVE, LOW);
  }

  delay(20);
}
