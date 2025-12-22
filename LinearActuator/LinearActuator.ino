#include <Arduino.h>

const int EN_PIN  = 18;  // XY-MC10 EN / PWM
const int DIR_PIN = 19;  // XY-MC10 DIR

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Motor OFF at startup
  digitalWrite(EN_PIN, LOW);
}

void loop() {
  
  digitalWrite(DIR_PIN, HIGH); 
  digitalWrite(EN_PIN, LOW);  
  delay(3000);

 
  digitalWrite(EN_PIN, LOW);   
  delay(3000);

  
  digitalWrite(DIR_PIN, LOW);  
  digitalWrite(EN_PIN, HIGH);  
  delay(3000);

  
  digitalWrite(EN_PIN, LOW);
  delay(3000);
}
