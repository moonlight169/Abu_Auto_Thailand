#include "kinematics.h"

#include <math.h>

#include "gyro.h"
#include "robot_state.h"
#include "wheel_link.h"

VelocityMode velocityMode = VELOCITY_STOPPED;

float commandedVx = 0.0f;
float commandedVy = 0.0f;
float commandedWz = 0.0f;

float rampedVx = 0.0f;
float rampedVy = 0.0f;
float rampedWz = 0.0f;

void setTargets(float fl, float fr, float rl, float rr)
{
  targetRPM[FL] = constrain(fl, -MAX_RPM, MAX_RPM);
  targetRPM[FR] = constrain(fr, -MAX_RPM, MAX_RPM);
  targetRPM[RL] = constrain(rl, -MAX_RPM, MAX_RPM);
  targetRPM[RR] = constrain(rr, -MAX_RPM, MAX_RPM);

  commandActive =
      fabsf(targetRPM[FL]) >= 1.0f ||
      fabsf(targetRPM[FR]) >= 1.0f ||
      fabsf(targetRPM[RL]) >= 1.0f ||
      fabsf(targetRPM[RR]) >= 1.0f;
}

// Low-level Mecanum inverse kinematics.
// vxLocal and vyLocal are expressed in the robot/body frame.
void setLocalVelocity(float vxLocal, float vyLocal, float wz)
{
  // vxLocal, vyLocal: m/s, wz: rad/s
  // Wheel order: FL, FR, RL, RR
  constexpr float MPS_TO_RPM =
      60.0f / (2.0f * PI * WHEEL_RADIUS_M);

  float wheelRPM[WHEEL_COUNT] = {
      (vxLocal - vyLocal - L_SUM_M * wz) * MPS_TO_RPM,
      (vxLocal + vyLocal + L_SUM_M * wz) * MPS_TO_RPM,
      (vxLocal + vyLocal - L_SUM_M * wz) * MPS_TO_RPM,
      (vxLocal - vyLocal + L_SUM_M * wz) * MPS_TO_RPM};

  // Scale every wheel by the same ratio. This preserves the requested
  // movement direction when one or more wheels exceed MAX_RPM.
  float largestRPM = 0.0f;
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
    largestRPM = max(largestRPM, fabsf(wheelRPM[wheel]));

  if (largestRPM > MAX_RPM)
  {
    const float scale = MAX_RPM / largestRPM;
    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
      wheelRPM[wheel] *= scale;
  }

  setTargets(
      wheelRPM[FL], wheelRPM[FR],
      wheelRPM[RL], wheelRPM[RR]);
}

// Field-centric command. Rotate the requested world-frame velocity into the
// robot/body frame, then use the same inverse kinematics as local control.
void setGlobalVelocity(float vxGlobal, float vyGlobal, float wz)
{
  const float yaw = gyroOnline
      ? gyroHeadingRad
      : encoderHeadingRad;

  const float cosYaw = cosf(yaw);
  const float sinYaw = sinf(yaw);

  // Global/world -> Local/robot
  const float vxLocal =
      vxGlobal * cosYaw + vyGlobal * sinYaw;
  const float vyLocal =
     -vxGlobal * sinYaw + vyGlobal * cosYaw;

  setLocalVelocity(vxLocal, vyLocal, wz);
}

void commandLocalVelocity(float vx, float vy, float wz)
{
  commandedVx = vx;
  commandedVy = vy;
  commandedWz = wz;
  velocityMode = VELOCITY_LOCAL;
}

void commandGlobalVelocity(float vx, float vy, float wz)
{
  commandedVx = vx;
  commandedVy = vy;
  commandedWz = wz;
  velocityMode = VELOCITY_GLOBAL;
}

float rampValue(float current, float target,
                float accelRate, float decelRate,
                float dt)
{
  // Changing direction must first decelerate through zero.
  const bool reversing =
      current * target < 0.0f;
  const bool accelerating =
      !reversing && fabsf(target) > fabsf(current);
  const float rate = accelerating ? accelRate : decelRate;
  const float maxChange = rate * dt;
  const float error = target - current;

  if (fabsf(error) <= maxChange)
    return target;

  return current + (error > 0.0f ? maxChange : -maxChange);
}

bool rampIsStopped()
{
  return fabsf(rampedVx) <= RAMP_ZERO_EPSILON &&
         fabsf(rampedVy) <= RAMP_ZERO_EPSILON &&
         fabsf(rampedWz) <= RAMP_ZERO_EPSILON;
}

void updateSoftVelocity(float targetVx,
                        float targetVy,
                        float targetWz,
                        bool globalMode)
{
  const float dt = COMMAND_PERIOD_MS * 0.001f;

  rampedVx = rampValue(
      rampedVx, targetVx,
      LINEAR_ACCEL, LINEAR_DECEL, dt);
  rampedVy = rampValue(
      rampedVy, targetVy,
      LINEAR_ACCEL, LINEAR_DECEL, dt);
  rampedWz = rampValue(
      rampedWz, targetWz,
      ANGULAR_ACCEL, ANGULAR_DECEL, dt);

  if (globalMode)
    setGlobalVelocity(rampedVx, rampedVy, rampedWz);
  else
    setLocalVelocity(rampedVx, rampedVy, rampedWz);

  sendTargetPacket();

  // A zero V/G command is a normal soft stop. Stop refreshing commands only
  // after every ramped axis has actually reached zero.
  const bool zeroTarget =
      fabsf(targetVx) <= RAMP_ZERO_EPSILON &&
      fabsf(targetVy) <= RAMP_ZERO_EPSILON &&
      fabsf(targetWz) <= RAMP_ZERO_EPSILON;

  if (zeroTarget && rampIsStopped())
  {
    velocityMode = VELOCITY_STOPPED;
    commandActive = false;
    Serial2.println("S");
    Serial.println("SOFT STOP COMPLETE");
  }
}

// Call after readGyro(). In GLOBAL mode this recalculates wheel RPM using
// the newest yaw every control period, even when no new USB command arrives.
void updateVelocityControl()
{
  if (velocityMode == VELOCITY_STOPPED)
    return;

  if (velocityMode == VELOCITY_GLOBAL)
    updateSoftVelocity(
        commandedVx, commandedVy, commandedWz, true);
  else
    updateSoftVelocity(
        commandedVx, commandedVy, commandedWz, false);
}

void stopRobot()
{
  velocityMode = VELOCITY_STOPPED;
  commandedVx = 0.0f;
  commandedVy = 0.0f;
  commandedWz = 0.0f;
  rampedVx = 0.0f;
  rampedVy = 0.0f;
  rampedWz = 0.0f;
  commandActive = false;
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
    targetRPM[wheel] = 0.0f;

  Serial2.println("S");
  Serial2.println("S");
  Serial2.println("S");
  lastCommandSendMs = millis();
}

float getRobotYawRad()
{
  return gyroOnline ? gyroHeadingRad : encoderHeadingRad;
}

void limitGlobalVelocity(float &vx, float &vy, float maxSpeed)
{
  const float speed = sqrtf(vx * vx + vy * vy);
  if (speed > maxSpeed && speed > 0.0001f)
  {
    const float scale = maxSpeed / speed;
    vx *= scale;
    vy *= scale;
  }
}
