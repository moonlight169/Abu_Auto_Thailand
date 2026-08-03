#pragma once

#include "config.h"
#include "position_pid.h"

// Homing and dual-axis position control.
//
//  beginHome()   drives both columns down at HOME_PWM until each BOTTOM limit
//                closes, zeroes that encoder, then reports LIFT_HOME_REACHED.
//  setTarget()   starts a position move and reports LIFT_BUSY, then
//                LIFT_REACHED once both columns stay inside the tolerance for
//                REACHED_CONFIRM_CYCLES.
//  updateControl() must be called from loop(); it runs every
//                CONTROL_PERIOD_MS and owns every motor write.

extern int32_t targetFront;
extern int32_t targetBack;
extern float setpointFront;
extern float setpointBack;
extern bool pidEnabled;
extern bool homing;
extern int frontPWM;
extern int backPWM;

void resetControllers();
void stopMotors();

void beginHome();
void setTarget(int32_t frontTarget, int32_t backTarget);
void zeroEncodersAndTargets();

void updateControl();
