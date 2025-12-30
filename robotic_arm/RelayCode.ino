#include <Bluepad32.h>

// ================== PINS ==================  ARE GOIN TO BE CHANGED 
#define RELAY1_PIN 26
#define RELAY2_PIN 27

// ================== GLOBALS ==================
ControllerPtr activeController = nullptr;

// Verrou logique (une seule transition possible)
bool switchedToRelay2 = false;

// ================== RELAY HELPERS (ACTIVE HIGH) ==================
void relay1On() {
  digitalWrite(RELAY1_PIN, HIGH);
}

void relay1Off() {
  digitalWrite(RELAY1_PIN, LOW);
}

void relay2On() {
  digitalWrite(RELAY2_PIN, HIGH);
}

void relay2Off() {
  digitalWrite(RELAY2_PIN, LOW);
}

// ================== CALLBACKS ==================
void onConnectedController(ControllerPtr ctl) {
  if (activeController == nullptr) {
    activeController = ctl;
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (activeController == ctl) {
    activeController = nullptr;
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  // --- Sécurité BOOT ---
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  // FORCER TOUT OFF AU DÉMARRAGE
  relay1Off();
  relay2Off();

  delay(100); // laisse l’ESP32 se stabiliser

  // État initial voulu
  relay1On();    // Relais 1 actif par défaut
  relay2Off();   // Relais 2 désactivé

  // Bluepad32
  Bluepad32.setup(&onConnectedController, &onDisconnectedController);
  Bluepad32.forgetBluetoothKeys();

  Serial.println("System ready: Relay 1 ON, Relay 2 OFF");
}

// ================== LOOP ==================
void loop() {
  Bluepad32.update();

  if (!activeController) return;

  // Bouton CIRCLE (⭕)
  if (activeController->buttons() & BUTTON_CIRCLE) {

    // Transition UNIQUE
    if (!switchedToRelay2) {
      switchedToRelay2 = true;

      // Séquence sécurisée (JAMAIS les deux ON)
      relay1Off();
      delay(30);      // marge de sécurité
      relay2On();

      Serial.println("Switched permanently to Relay 2");
    }
  }
}

