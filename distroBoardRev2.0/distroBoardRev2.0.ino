/* * Distro Board Rev 2.0 - ATtiny404
 * Logic for Precharge and Contactor Control
 */

// --- Pin Definitions (Adjust based on your PCB layout) ---
const int PIN_LED        = PIN_PA7;  // PA7
const int PIN_CONTACTOR  = PIN_PA5;  // PA5
const int PIN_PRECHARGE  = PIN_PA4;  // PA4
const int PIN_ESTOP      = PIN_PA6;  // PA6 (Low = Fault)
const int PIN_LOAD_V     = PIN_PA1; // ADC pin

// --- Constants & Calibration ---
const float V_REF          = 4.3;     // Internal or VCC ref
const float DIVIDER_RATIO  = 26.0;    // Example: 100k/5k divider (Scale to your PCB!)
const float MIN_V_READY    = 60.0;    // Target voltage to close contactor
const float MAX_DV_DT      = 1.0;     // Max 1V/s change to consider "settled"
const unsigned long TIMEOUT_MS = 10000; // 10s Precharge timeout

// --- Global Variables ---
float initialLoadVoltage = 0;
bool faultActive = false;

// --- Function Prototypes ---
float readVoltage();
void throwFault(String reason);
void checkEstop();

void setup() {
  // 1. Initial State: Safety First
  pinMode(PIN_CONTACTOR, OUTPUT);
  pinMode(PIN_PRECHARGE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ESTOP, INPUT); // Ensure pullup if not on PCB

  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
  digitalWrite(PIN_LED, LOW);

  // 2. Setup ADC & Read Initial State
  analogReference(INTERNAL4V3); // can use 4.3
  initialLoadVoltage = readVoltage();

  // 3. Initial Fault Check
  checkEstop();

  if (!faultActive) {
    beginPrecharge();
  }
}

void loop() {
  // Post-precharge monitor
  checkEstop();
  
  if (faultActive) {
    digitalWrite(PIN_CONTACTOR, LOW);
    digitalWrite(PIN_PRECHARGE, LOW);
    // Rapid blink LED for fault
    digitalWrite(PIN_LED, (millis() / 100) % 2); 
  }
}

float readVoltage() {
  int raw = analogRead(PIN_LOAD_V);
  // Formula: Raw * (Ref / Resolution) * Divider
  return (raw * (V_REF / 1023.0)) * DIVIDER_RATIO;
}

void checkEstop() {
  if (digitalRead(PIN_ESTOP) == LOW) {
    throwFault("ESTOP TRIGGERED");
  }
}

void beginPrecharge() {
  unsigned long startTime = millis();
  float lastVoltage = readVoltage();
  unsigned long lastCheckTime = startTime;

  digitalWrite(PIN_PRECHARGE, HIGH);

  while (millis() - startTime < TIMEOUT_MS) {
    checkEstop();
    if (faultActive) return;

    delay(200); // Sample rate
    float currentVoltage = readVoltage();
    float dt = (millis() - lastCheckTime) / 1000.0;
    float dvdt = (currentVoltage - lastVoltage) / dt;

    // Condition to close contactor:
    // > 60V AND > 1s elapsed AND voltage has settled (dv/dt < 1V/s)
    if (currentVoltage > MIN_V_READY && (millis() - startTime > 1000) && dvdt < MAX_DV_DT) {
      digitalWrite(PIN_CONTACTOR, HIGH);
      delay(100); // 100ms overlap
      digitalWrite(PIN_PRECHARGE, LOW);
      digitalWrite(PIN_LED, HIGH); // Success Indicator
      return; 
    }

    lastVoltage = currentVoltage;
    lastCheckTime = millis();
  }

  // If we exit the loop without returning, it's a timeout fault
  float finalV = readVoltage();
  float finalDt = (millis() - lastCheckTime) / 1000.0;
  float finalDvdt = (finalV - lastVoltage) / finalDt;

  if (finalV < MIN_V_READY || finalDvdt > MAX_DV_DT) {
    throwFault("PRECHARGE TIMEOUT / INSTABILITY");
  }
}

void throwFault(String reason) {
  faultActive = true;
  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
  // Logic stops here; loop() handles the blink
}