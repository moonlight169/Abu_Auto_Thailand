#include "io_board.h"

#include <Servo.h>

static Servo armServo;
static Servo spinServo;

int armAngleDeg = ARM_MIN_DEG;
int spinAngleDeg = SPIN_MIN_DEG;

InputChannel inputs[] = {
  {"L_SW_Front", L_SW_Front, HIGH, HIGH, 0},
  {"LDR1",       LDR1,       HIGH, HIGH, 0},
  {"LDR2",       LDR2,       HIGH, HIGH, 0},
  {"laser5",     laser5_sen, HIGH, HIGH, 0},
  {"SW_Green",   SW_Green,   HIGH, HIGH, 0},
  {"SW_Blue",    SW_Blue,    HIGH, HIGH, 0},
  {"SW_Red",     SW_Red,     HIGH, HIGH, 0},
  {"SW_Yellow",  SW_Yellow,  HIGH, HIGH, 0},
  // Keep this input last so the existing inputMask bit mapping stays unchanged.
  {"R_SW_Front", R_SW_Front, HIGH, HIGH, 0}
};

const uint8_t INPUT_COUNT = sizeof(inputs) / sizeof(inputs[0]);

void writeRelay(uint8_t pin, bool on) {
  digitalWrite(pin, RELAY_ACTIVE_HIGH ? (on ? HIGH : LOW)
                                      : (on ? LOW : HIGH));
}

bool readRelayState(uint8_t pin) {
  const bool level = digitalRead(pin);
  return RELAY_ACTIVE_HIGH ? level == HIGH : level == LOW;
}

void setRelay(uint8_t relayNumber, bool on) {
  uint8_t pin;
  switch (relayNumber) {
    case 1: pin = relay1; break;
    case 2: pin = relay2; break;
    case 3: pin = relay3; break;
    case 4: pin = relay4; break;
    default:
      Serial.println("ERROR: Relay number must be 1-4");
      return;
  }
  writeRelay(pin, on);
  Serial.print("RELAY");
  Serial.print(relayNumber);
  Serial.print('=');
  Serial.println(on ? "ON" : "OFF");
}

void setAllRelays(bool on) {
  writeRelay(relay1, on);
  writeRelay(relay2, on);
  writeRelay(relay3, on);
  writeRelay(relay4, on);
  Serial.print("ALL RELAYS=");
  Serial.println(on ? "ON" : "OFF");
}

void setArmServo(int angleDeg) {
  armAngleDeg = constrain(angleDeg, ARM_MIN_DEG, ARM_MAX_DEG);
  armServo.write(armAngleDeg);
}

void setSpinServo(int angleDeg) {
  spinAngleDeg = constrain(angleDeg, SPIN_MIN_DEG, SPIN_MAX_DEG);
  spinServo.write(spinAngleDeg);
}

static void initializeInputs() {
  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    pinMode(inputs[i].pin, INPUT_PULLUP);
    const bool state = digitalRead(inputs[i].pin);
    inputs[i].stableState = state;
    inputs[i].lastRawState = state;
    inputs[i].changedTime = millis();
  }
}

void initializeIoBoard() {
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  setRelay(1, 0);
  setRelay(2, 1);
  setRelay(3, 1);
  setRelay(4, 0);

  initializeInputs();
  armServo.attach(arm_servo);
  spinServo.attach(spin_servo);
  armServo.write(armAngleDeg);
  spinServo.write(spinAngleDeg);
}

// Debounce is maintained internally without continuously printing changes.
void updateInputs() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    const bool rawState = digitalRead(inputs[i].pin);
    if (rawState != inputs[i].lastRawState) {
      inputs[i].lastRawState = rawState;
      inputs[i].changedTime = now;
    }
    if ((now - inputs[i].changedTime) >= INPUT_DEBOUNCE_MS &&
        rawState != inputs[i].stableState) {
      inputs[i].stableState = rawState;
    }
  }
}

uint16_t getInputActiveMask() {
  uint16_t activeMask = 0;
  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    if (inputs[i].stableState == LOW) {
      activeMask |= (uint16_t(1) << i);
    }
  }
  return activeMask;
}

uint8_t getRelayOnMask() {
  uint8_t relayMask = 0;
  if (readRelayState(relay1)) relayMask |= (1U << 0);
  if (readRelayState(relay2)) relayMask |= (1U << 1);
  if (readRelayState(relay3)) relayMask |= (1U << 2);
  if (readRelayState(relay4)) relayMask |= (1U << 3);
  return relayMask;
}

void printAllInputs() {
  Serial.println("---------- INPUT STATUS ----------");
  for (uint8_t i = 0; i < INPUT_COUNT; i++) {
    Serial.print(inputs[i].name);
    Serial.print(" [Pin ");
    Serial.print(inputs[i].pin);
    Serial.print("] RAW=");
    Serial.print(inputs[i].stableState ? 1 : 0);
    Serial.print(" STATE=");
    Serial.println(inputs[i].stableState == LOW ? "ACTIVE" : "INACTIVE");
  }
}

void printAllOutputs() {
  Serial.println("---------- OUTPUT STATUS ---------");
  Serial.print("Relay1: ");
  Serial.println(readRelayState(relay1) ? "ON" : "OFF");
  Serial.print("Relay2: ");
  Serial.println(readRelayState(relay2) ? "ON" : "OFF");
  Serial.print("Relay3: ");
  Serial.println(readRelayState(relay3) ? "ON" : "OFF");
  Serial.print("Relay4: ");
  Serial.println(readRelayState(relay4) ? "ON" : "OFF");
  Serial.print("Arm servo angle: ");
  Serial.println(armServo.read());
  Serial.print("Spin servo angle: ");
  Serial.println(spinServo.read());
}

void printStatus() {
  printAllInputs();
  printAllOutputs();
  Serial.println("---------------------------------");
}
