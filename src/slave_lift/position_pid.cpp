#include "position_pid.h"

#include <math.h>

float frontKp = FRONT_KP;
float frontKi = FRONT_KI;
float frontKd = FRONT_KD;
float backKp = BACK_KP;
float backKi = BACK_KI;
float backKd = BACK_KD;

void PositionPID::reset() {
  integral = 0.0f;
  previousError = 0.0f;
  first = true;
}

float PositionPID::compute(float setpoint, int32_t position, float dt,
                           float kp, float ki, float kd) {
  const float error = setpoint - (float)position;
  const float derivative =
      first ? 0.0f : (error - previousError) / dt;
  first = false;
  previousError = error;

  const float candidateIntegral = integral + error * dt;
  const float candidateOutput =
      kp * error + ki * candidateIntegral + kd * derivative;

  // Conditional integration anti-windup.
  if ((candidateOutput < PWM_MAX &&
       candidateOutput > -PWM_MAX) ||
      (candidateOutput >= PWM_MAX && error < 0.0f) ||
      (candidateOutput <= -PWM_MAX && error > 0.0f)) {
    integral = candidateIntegral;
  }

  if (ki > 0.0f) {
    const float integralLimit = (float)PWM_MAX / ki;
    integral = constrain(integral, -integralLimit, integralLimit);
  } else {
    integral = 0.0f;
  }

  return kp * error + ki * integral + kd * derivative;
}

int calculatePositionPWM(PositionPID &pid, float setpoint, int32_t position,
                         float dt, float kp, float ki, float kd) {
  const float error = setpoint - (float)position;

  // Prevent direction hunting close to the target.
  if (fabsf(error) <= POSITION_TOLERANCE_PULSE) {
    pid.reset();
    return 0;
  }

  const float effort = pid.compute(setpoint, position, dt, kp, ki, kd);

  // Positive encoder direction is up. Negative PWM drives up.
  const int pwm = -(int)lroundf(effort);
  return constrain(pwm, -PWM_MAX, PWM_MAX);
}

int applyDownSoftPWM(int targetPWM, int currentPWM,
                     int32_t targetPosition, int32_t currentPosition,
                     int downPWMMax) {
  // Negative PWM is upward: apply PID output directly with no ramp.
  if (targetPWM < 0) return targetPWM;

  // Starting a downward move after upward PWM begins from zero.
  if (currentPWM < 0) currentPWM = 0;

  // Limit downward power. Inside the braking zone the allowed PWM falls
  // linearly with the remaining distance, so the lift slows before target.
  const int32_t remaining =
      max((int32_t)0, currentPosition - targetPosition);
  int downLimit = downPWMMax;
  if (remaining < DOWN_BRAKE_ZONE_PULSE) {
    downLimit = (int)((int64_t)downPWMMax * remaining /
                      DOWN_BRAKE_ZONE_PULSE);
  }
  targetPWM = constrain(targetPWM, 0, downLimit);

  if (targetPWM > currentPWM) {
    return min(targetPWM, currentPWM + DOWN_ACCEL_STEP);
  }

  if (targetPWM < currentPWM) {
    return max(targetPWM, currentPWM - DOWN_DECEL_STEP);
  }

  return targetPWM;
}
