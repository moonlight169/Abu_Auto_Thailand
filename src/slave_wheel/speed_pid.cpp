#include "speed_pid.h"

#include "encoder.h"
#include "motor.h"

float targetRPM[WHEEL_COUNT] = {0, 0, 0, 0};
int16_t pwmOutput[WHEEL_COUNT] = {0, 0, 0, 0};
bool driveEnabled = false;
uint32_t lastCommandTime = 0;

static float integral[WHEEL_COUNT] = {0, 0, 0, 0};
static float previousError[WHEEL_COUNT] = {0, 0, 0, 0};

void resetAllControllers()
{
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    integral[wheel] = 0.0f;
    previousError[wheel] = 0.0f;
    resetRPMFilter(wheel);
  }
}

void updateSpeedPID(uint8_t wheel)
{
  if (!driveEnabled || fabsf(targetRPM[wheel]) < 1.0f)
  {
    integral[wheel] = 0.0f;
    previousError[wheel] = 0.0f;
    pwmOutput[wheel] = 0;
    setMotorPWM(wheel, 0);
    return;
  }

  const float error = targetRPM[wheel] - actualRPM[wheel];
  const float derivative =
      (error - previousError[wheel]) / CONTROL_DT;

  const float candidateIntegral =
      integral[wheel] + error * CONTROL_DT;
  const float candidateOutput =
      kp[wheel] * error +
      ki[wheel] * candidateIntegral +
      kd[wheel] * derivative;

  const bool saturatedHigh = candidateOutput > PWM_MAX;
  const bool saturatedLow = candidateOutput < PWM_MIN;
  if ((!saturatedHigh && !saturatedLow) ||
      (saturatedHigh && error < 0.0f) ||
      (saturatedLow && error > 0.0f))
  {
    integral[wheel] = candidateIntegral;
  }

  const float integralLimit = 255.0f / max(ki[wheel], 0.001f);
  integral[wheel] =
      constrain(integral[wheel], -integralLimit, integralLimit);

  const float output =
      kp[wheel] * error +
      ki[wheel] * integral[wheel] +
      kd[wheel] * derivative;

  previousError[wheel] = error;
  pwmOutput[wheel] =
      (int16_t)constrain(output, (float)PWM_MIN, (float)PWM_MAX);
  setMotorPWM(wheel, pwmOutput[wheel]);
}

void stopDrive()
{
  driveEnabled = false;

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    targetRPM[wheel] = 0.0f;
    integral[wheel] = 0.0f;
    previousError[wheel] = 0.0f;
    pwmOutput[wheel] = 0;
    setMotorPWM(wheel, 0);
  }
}

void setWheelTargets(float fl, float fr, float rl, float rr)
{
  const float requestedRPM[WHEEL_COUNT] = {fl, fr, rl, rr};

  bool anyWheelMoving = false;
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    const float newTarget =
        constrain(requestedRPM[wheel], -MAX_RPM, MAX_RPM);

    if ((targetRPM[wheel] > 0.0f && newTarget < 0.0f) ||
        (targetRPM[wheel] < 0.0f && newTarget > 0.0f))
    {
      integral[wheel] = 0.0f;
      previousError[wheel] = 0.0f;
    }

    targetRPM[wheel] = newTarget;
    if (fabsf(newTarget) >= 1.0f)
      anyWheelMoving = true;
  }

  driveEnabled = anyWheelMoving;
  lastCommandTime = millis();

  if (!driveEnabled)
    stopDrive();
}
