#pragma once

#include "config.h"

// Low-level H-bridge output. Directional limit safety is always applied here,
// so no other module can drive an axis into a pressed end stop.

void initializeMotorPins();

bool isPressed(uint32_t pin);

void motorWrite(uint32_t pinA, uint32_t pinB, int pwm);
void bottomMotor(int pwm);
void topMotor(int pwm);
