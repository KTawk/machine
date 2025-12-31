// ================= CONFIG =================
#define RELAY_PIN  25    // GPIO connected to IN1
//#define RELAY_ON   HIGH  // HIGH-level trigger
//#define RELAY_OFF  LOW

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Relay OFF at boot
}

void loop() {
  // Turn relay ON
  digitalWrite(RELAY_PIN, HIGH);
  delay(2000);

//   // Turn relay OFF
//   digitalWrite(RELAY_PIN, RELAY_OFF);
//   delay(2000);
 }
