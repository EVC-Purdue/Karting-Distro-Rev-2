/* * Distro Board Rev 2.0 - ATtiny404
 * Logic for Precharge and Contactor Control
 */

#include <Arduino.h>
#include <avr/interrupt.h>

// --- Pin Definitions (Adjust based on your PCB layout) ---
const int PIN_LED        = PIN_PA7;  // PA7
const int PIN_CONTACTOR  = PIN_PA5;  // PA5
const int PIN_PRECHARGE  = PIN_PA4;  // PA4
const int PIN_ESTOP      = PIN_PA6;  // PA6 (Low = Fault)
const int PIN_LOAD_V     = PIN_PA1; // ADC pin

// --- Constants & Calibration ---
const float V_REF          = 4.3;     // Internal or VCC ref
const float DIVIDER_RATIO  = 26.0;    
const float MIN_V_READY    = 60.0;    // Target voltage to close contactor
const float MAX_DV_DT      = 1.0;     // Max 1V/s change to consider "settled"
const unsigned long TIMEOUT_MS = 10000; // 10s Precharge timeout

const uint16_t LED_TIMER_TICK_MS = 10;
const uint16_t LED_SLOW_BLINK_MS = 500;
const uint16_t LED_FAST_BLINK_MS = 100;

enum FaultType : uint8_t {
  FAULT_NONE = 0,
  FAULT_ESTOP,
  FAULT_PRECHARGE_FAIL,
  FAULT_CONTACTOR_WELD
};

enum LedMode : uint8_t {
  LED_OFF = 0,
  LED_SOLID,
  LED_BLINK_SLOW,
  LED_BLINK_FAST
};

// --- Global Variables ---
float initialLoadVoltage = 0;
volatile bool faultActive = false;
volatile bool estopInterruptFlag = false;
volatile FaultType activeFault = FAULT_NONE;

volatile LedMode ledMode = LED_OFF;
volatile bool ledOutputState = false;
volatile uint16_t ledTickCounter = 0;

// --- Function Prototypes ---
float readVoltage();
void throwFault(FaultType type);
void updateLedModeForFault(FaultType type);
void setupLedTimer();
void setupEstopInterrupt();
void applyFaultOutputs();
void handleEstopLatched();
void beginPrecharge();

ISR(TCB0_INT_vect) {
  if (ledMode == LED_SOLID) {
    ledOutputState = true;
  } else if (ledMode == LED_OFF) {
    ledOutputState = false;
  } else {
    const uint16_t targetMs = (ledMode == LED_BLINK_FAST) ? LED_FAST_BLINK_MS : LED_SLOW_BLINK_MS;
    const uint16_t targetTicks = targetMs / LED_TIMER_TICK_MS;

    ledTickCounter++;
    if (ledTickCounter >= targetTicks) {
      ledTickCounter = 0;
      ledOutputState = !ledOutputState;
    }
  }

  digitalWrite(PIN_LED, ledOutputState ? HIGH : LOW);
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

void onEstopTriggered() {
  // Filter checks: ensure pin stays LOW for ~1ms to rule out switching noise
  // This prevents the contactor's own switching EMI from tripping the sensitive interrupt
  for (int i = 0; i < 1000; i++) {
    if (digitalRead(PIN_ESTOP) == HIGH) {
      return; // False alarm - pin returned high (safe state)
    }
    delayMicroseconds(100);
  }

  estopInterruptFlag = true;

  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
}

void setup() {
  // 1. Initial State: Safety First
  pinMode(PIN_CONTACTOR, OUTPUT);
  pinMode(PIN_PRECHARGE, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_ESTOP, INPUT); // Ensure pullup if not on PCB

  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
  digitalWrite(PIN_LED, LOW);

  setupLedTimer();
  
  // 2. Setup ADC & Read Initial State
  analogReference(INTERNAL4V3); // can use 4.3
  initialLoadVoltage = readVoltage();

  // 3. Initial Fault Check
  delay(1000);
  if (digitalRead(PIN_ESTOP) == LOW) {
    throwFault(FAULT_ESTOP);
  }

  // Initialize interrupt monitoring *after* confirming stable state
  setupEstopInterrupt();
  estopInterruptFlag = false; 

  if (!faultActive && initialLoadVoltage > MIN_V_READY) {
    throwFault(FAULT_CONTACTOR_WELD);
  }

  if (!faultActive) {
    beginPrecharge();
  }
}

void loop() {
  handleEstopLatched();

  if (faultActive) {
    applyFaultOutputs();
  }

  delay(100); // Reduce CPU load
}

void setupLedTimer() {
  noInterrupts();

  TCB0.CTRLA = 0;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = (uint16_t)((F_CPU / 2UL / 100UL) - 1UL);
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CNT = 0;
  TCB0.CTRLA = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;

  interrupts();
}

void setupEstopInterrupt() {
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), onEstopTriggered, FALLING);
}

float readVoltage() {
  int raw = analogRead(PIN_LOAD_V);
  // Formula: Raw * (Ref / Resolution) * Divider
  return (raw * (V_REF / 1023.0)) * DIVIDER_RATIO;
}

void applyFaultOutputs() {
  digitalWrite(PIN_CONTACTOR, LOW);
  digitalWrite(PIN_PRECHARGE, LOW);
}

void updateLedModeForFault(FaultType type) {
  noInterrupts();
  ledTickCounter = 0;

  if (type == FAULT_ESTOP) {
    ledMode = LED_SOLID;
    ledOutputState = true;
  } else if (type == FAULT_PRECHARGE_FAIL) {
    ledMode = LED_BLINK_SLOW;
  } else if (type == FAULT_CONTACTOR_WELD) {
    ledMode = LED_BLINK_FAST;
  } else {
    ledMode = LED_OFF;
    ledOutputState = false;
  }

  interrupts();
}

void handleEstopLatched() {
  if (estopInterruptFlag) {
    estopInterruptFlag = false;
    throwFault(FAULT_ESTOP);
  }
}

void beginPrecharge() {
  unsigned long startTime = millis();
  float lastVoltage = readVoltage();
  unsigned long lastCheckTime = startTime;

  digitalWrite(PIN_PRECHARGE, HIGH);

  while (millis() - startTime < TIMEOUT_MS) {
    handleEstopLatched();
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
    throwFault(FAULT_PRECHARGE_FAIL);
  }
}

void throwFault(FaultType type) {
  if (!faultActive) {
    faultActive = true;
    activeFault = type;
    applyFaultOutputs();
    updateLedModeForFault(type);
    return;
  }

  // ESTOP always takes priority over any existing fault indication.
  if (type == FAULT_ESTOP && activeFault != FAULT_ESTOP) {
    activeFault = FAULT_ESTOP;
    applyFaultOutputs();
    updateLedModeForFault(FAULT_ESTOP);
  }
}