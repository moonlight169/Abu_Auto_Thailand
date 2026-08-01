#pragma once

#include "config.h"

// Quadrature encoder counting + moving-average RPM measurement.

extern volatile int32_t encoderCount[WHEEL_COUNT];
extern int32_t previousEncoderCount[WHEEL_COUNT];

extern float rawRPM[WHEEL_COUNT];
extern float actualRPM[WHEEL_COUNT];

void attachEncoderInterrupts();

void resetRPMFilter(uint8_t wheel);
void updateRPM(uint8_t wheel);
