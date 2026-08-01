#pragma once

#include "config.h"

// Per-wheel speed PID with anti-windup, plus the target/enable bookkeeping.

extern float targetRPM[WHEEL_COUNT];
extern int16_t pwmOutput[WHEEL_COUNT];
extern bool driveEnabled;
extern uint32_t lastCommandTime;

void resetAllControllers();
void updateSpeedPID(uint8_t wheel);
void stopDrive();
void setWheelTargets(float fl, float fr, float rl, float rr);
