#include "position_control.h"

#include <math.h>

#include "kinematics.h"
#include "robot_state.h"
#include "tasks.h"
#include "wheel_link.h"

bool positionControlActive = false;

float targetPositionX = 0.0f;
float targetPositionY = 0.0f;
float targetPositionYawRad = 0.0f;

float previousErrorX = 0.0f;
float previousErrorY = 0.0f;
float previousErrorYaw = 0.0f;

uint8_t positionDoneCounter = 0;
float currentMoveMaxSpeed = MAX_POSITION_SPEED;

static uint32_t lastPositionControlMs = 0;

void setPositionTarget(float x, float y, float yawDeg)
{
  targetPositionX = x;
  targetPositionY = y;
  targetPositionYawRad = normalizeAngle(yawDeg * DEG_TO_RAD);

  previousErrorX = targetPositionX - positionX;
  previousErrorY = targetPositionY - positionY;
  previousErrorYaw =
      normalizeAngle(targetPositionYawRad - getRobotYawRad());

  velocityMode = VELOCITY_STOPPED;
  commandedVx = 0.0f;
  commandedVy = 0.0f;
  commandedWz = 0.0f;
  positionDoneCounter = 0;
  lastPositionControlMs = millis();
  positionControlActive = true;
  commandActive = true;

  Serial.print("POSITION TARGET X=");
  Serial.print(x, 3);
  Serial.print(" Y=");
  Serial.print(y, 3);
  Serial.print(" Yaw=");
  Serial.println(yawDeg, 2);
}

void startMoveTo(float x, float y, float yawDeg,
                 uint32_t timeoutMs, float maxSpeed)
{
  moveTaskStartMs = millis();
  moveTaskTimeoutMs = timeoutMs;
  moveTaskStatus = TASK_RUNNING;

  currentMoveMaxSpeed = constrain(
      maxSpeed,
      0.01f,
      MAX_POSITION_SPEED);

  Serial.print("MOVE MAX SPEED=");
  Serial.println(currentMoveMaxSpeed, 3);

  setPositionTarget(x, y, yawDeg);
}

bool moveDone()
{
  return moveTaskStatus == TASK_DONE;
}

bool moveRunning()
{
  return moveTaskStatus == TASK_RUNNING;
}

bool moveFailed()
{
  return moveTaskStatus == TASK_TIMEOUT ||
         moveTaskStatus == TASK_ERROR;
}

void cancelPositionControl()
{
  positionControlActive = false;
  positionDoneCounter = 0;
  if (moveTaskStatus == TASK_RUNNING)
    moveTaskStatus = TASK_ERROR;
  stopRobot();
  Serial.println("POSITION CANCELLED");
}

void updatePositionControl()
{
  if (!positionControlActive)
    return;

  const uint32_t now = millis();
  const uint32_t elapsedMs =
      (uint32_t)(now - lastPositionControlMs);

  if (elapsedMs < POSITION_PERIOD_MS)
    return;

  float dt = elapsedMs * 0.001f;
  lastPositionControlMs = now;

  if (dt <= 0.0f || dt > 0.2f)
    dt = POSITION_PERIOD_MS * 0.001f;

  const float currentYaw = getRobotYawRad();

  const float errorX =
      targetPositionX - positionX;

  const float errorY =
      targetPositionY - positionY;

  const float errorYaw =
      normalizeAngle(targetPositionYawRad - currentYaw);

  const bool xReached =
      fabsf(errorX) <= POSITION_TOLERANCE_M;

  const bool yReached =
      fabsf(errorY) <= POSITION_TOLERANCE_M;

  const bool yawReached =
      fabsf(errorYaw) <= YAW_TOLERANCE_RAD;

  if (xReached && yReached && yawReached)
  {
    if (positionDoneCounter < POSITION_DONE_COUNT)
      positionDoneCounter++;

    setLocalVelocity(0.0f, 0.0f, 0.0f);
    sendTargetPacket();

    if (positionDoneCounter >= POSITION_DONE_COUNT)
    {
      positionControlActive = false;
      positionDoneCounter = 0;

      stopRobot();

      if (moveTaskStatus == TASK_RUNNING)
        moveTaskStatus = TASK_DONE;

      Serial.print("POSITION REACHED X=");
      Serial.print(positionX, 3);

      Serial.print(" Y=");
      Serial.print(positionY, 3);

      Serial.print(" YAW=");
      Serial.println(currentYaw * RAD_TO_DEG, 2);
    }

    return;
  }

  positionDoneCounter = 0;

  const float derivativeX =
      (errorX - previousErrorX) / dt;

  const float derivativeY =
      (errorY - previousErrorY) / dt;

  const float derivativeYaw =
      normalizeAngle(errorYaw - previousErrorYaw) / dt;

  previousErrorX = errorX;
  previousErrorY = errorY;
  previousErrorYaw = errorYaw;

  float vxGlobal =
      KP_POSITION * errorX +
      KD_POSITION * derivativeX;

  float vyGlobal =
      KP_POSITION * errorY +
      KD_POSITION * derivativeY;

  float wz =
      KP_YAW * errorYaw +
      KD_YAW * derivativeYaw;

  if (xReached)
    vxGlobal = 0.0f;

  if (yReached)
    vyGlobal = 0.0f;

  if (yawReached)
    wz = 0.0f;

  limitGlobalVelocity(
      vxGlobal,
      vyGlobal,
      currentMoveMaxSpeed);

  wz = constrain(
      wz,
      -MAX_POSITION_WZ,
      MAX_POSITION_WZ);

  setGlobalVelocity(vxGlobal, vyGlobal, wz);
  sendTargetPacket();
}
