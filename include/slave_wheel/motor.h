#pragma once

#include "config.h"

// H-bridge output stage. PWM sign selects the direction.

void initializeMotorPins();
void setMotorPWM(uint8_t wheel, int16_t pwm);
