#pragma once

#include "config.h"

// Position PID with conditional-integration anti-windup, plus the soft ramp
// that shapes the raw PID output whenever a column travels downward.

struct PositionPID {
  float integral = 0.0f;
  float previousError = 0.0f;
  bool first = true;

  void reset();

  float compute(float setpoint, int32_t position, float dt,
                float kp, float ki, float kd);
};

// Live gains, seeded from config.h and editable from the console.
extern float frontKp;
extern float frontKi;
extern float frontKd;
extern float backKp;
extern float backKi;
extern float backKd;

// PID output converted to PWM. Returns 0 inside POSITION_TOLERANCE_PULSE so
// the column cannot hunt around the target.
int calculatePositionPWM(PositionPID &pid, float setpoint, int32_t position,
                         float dt, float kp, float ki, float kd);

// Upward PWM passes through untouched. Downward PWM is limited to
// downPWMMax, ramped by DOWN_ACCEL_STEP / DOWN_DECEL_STEP and faded out
// linearly across the last DOWN_BRAKE_ZONE_PULSE pulses.
int applyDownSoftPWM(int targetPWM, int currentPWM,
                     int32_t targetPosition, int32_t currentPosition,
                     int downPWMMax);
