#include "tasks.h"

#include "kinematics.h"
#include "lift_link.h"
#include "position_control.h"

TaskStatus moveTaskStatus = TASK_IDLE;
TaskStatus liftTaskStatus = TASK_IDLE;
TaskStatus boxParkingTaskStatus = TASK_IDLE;

uint32_t moveTaskStartMs = 0;
uint32_t moveTaskTimeoutMs = 0;
uint32_t liftTaskStartMs = 0;
uint32_t liftTaskTimeoutMs = 0;

void updateRobotTasks()
{
  const uint32_t now = millis();

  if (moveTaskStatus == TASK_RUNNING &&
      moveTaskTimeoutMs > 0 &&
      (uint32_t)(now - moveTaskStartMs) >= moveTaskTimeoutMs)
  {
    positionControlActive = false;
    stopRobot();
    moveTaskStatus = TASK_TIMEOUT;
    Serial.println("ERROR: MOVE TIMEOUT");
  }

  if (liftTaskStatus == TASK_RUNNING &&
      liftTaskTimeoutMs > 0 &&
      (uint32_t)(now - liftTaskStartMs) >= liftTaskTimeoutMs)
  {
    liftMoveActive = false;
    liftHomeActive = false;
    liftBusy = false;
    liftTaskStatus = TASK_TIMEOUT;
    Serial.print("ERROR: LIFT TIMEOUT targetF=");
    Serial.print(liftTargetFront);
    Serial.print(" targetB=");
    Serial.print(liftTargetBack);
    Serial.print(" lastF=");
    Serial.print(liftPosFront);
    Serial.print(" lastB=");
    Serial.println(liftPosBack);
  }
}
