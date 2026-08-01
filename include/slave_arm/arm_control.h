#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  The arm state machine: homing sequence, dual-axis PID position hold and
//  the manual PWM test modes. This is the only module that moves the arm.
// ---------------------------------------------------------------------------

extern ArmState state;
extern uint32_t stateStartMs;

extern bool bottomHomed;
extern bool topHomed;
extern bool bottomHolding;
extern bool topHolding;

extern float bottomTargetDeg;
extern float topTargetDeg;

extern int testPwm;

const __FlashStringHelper *stateName();

float getTopPositionDeg();
float getBottomPositionDeg();

void stopAll();
void enterFault(const __FlashStringHelper *message);

void startHome();
bool commandBottomPosition(float degrees);
bool commandTopPosition(float degrees);

void startBottomPwmTest(int pwm);
void startTopPwmTest(int pwm);

void updateStateMachine();
