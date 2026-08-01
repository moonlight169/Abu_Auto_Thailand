#pragma once

#include "config.h"

// Relays, debounced digital inputs and the two servos.

extern InputChannel inputs[];
extern const uint8_t INPUT_COUNT;

extern int armAngleDeg;
extern int spinAngleDeg;

void initializeIoBoard();

void writeRelay(uint8_t pin, bool on);
bool readRelayState(uint8_t pin);
void setRelay(uint8_t relayNumber, bool on);
void setAllRelays(bool on);

void setArmServo(int angleDeg);
void setSpinServo(int angleDeg);

void updateInputs();
uint16_t getInputActiveMask();
uint8_t getRelayOnMask();

void printAllInputs();
void printAllOutputs();
void printStatus();
