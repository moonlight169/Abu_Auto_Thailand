#include "lift_link.h"

#include <stdio.h>
#include <string.h>

#include "gyro.h"
#include "hub_link.h"
#include "tasks.h"
#include "wheel_link.h"

int32_t liftFrontCount = 0;
int32_t liftBackCount = 0;
int32_t liftTargetFront = 0;
int32_t liftTargetBack  = 0;
int32_t liftPosFront = 0;
int32_t liftPosBack  = 0;

bool liftBusy = false;
bool liftAtTarget = false;
bool liftHomeReached = false;
bool liftMoveActive = false;
bool liftHomeActive = false;
bool liftReached = false;

uint32_t liftResponseSequence = 0;

static char liftRxBuffer[LIFT_RX_SIZE];
static size_t liftRxIndex = 0;
static uint8_t liftConfirmCount = 0;

void sendLiftCommandPulse(int32_t front, int32_t back)
{
  Serial7.print("LP,");
  Serial7.print(front);
  Serial7.print(",");
  Serial7.println(back);

  Serial.print("LIFT PULSE SENT front=");
  Serial.print(front);
  Serial.print(" back=");
  Serial.println(back);
}

void sendLiftCommandZero()
{
  Serial7.println("LZ");
  Serial.println("LIFT ZERO/HOME SENT");
}

static bool parseLiftResponse(char *line)
{
  long front = 0;
  long back = 0;

  if (sscanf(line, "LIFT_POS,%ld,%ld", &front, &back) == 2)
  {
    liftPosFront = (int32_t)front;
    liftPosBack = (int32_t)back;
    liftFrontCount = liftPosFront;
    liftBackCount = liftPosBack;

    if (liftMoveActive)
    {
      const int32_t frontError =
          (int32_t)labs((long)liftTargetFront - front);
      const int32_t backError =
          (int32_t)labs((long)liftTargetBack - back);

      if (frontError <= LIFT_TOLERANCE &&
          backError <= LIFT_TOLERANCE)
      {
        if (liftConfirmCount < LIFT_CONFIRM_REQUIRED)
          liftConfirmCount++;

        if (liftConfirmCount >= LIFT_CONFIRM_REQUIRED)
        {
          liftReached = true;
          liftMoveActive = false;
          liftBusy = false;
          liftAtTarget = true;
          liftTaskStatus = TASK_DONE;

          Serial.print("LIFT_POSITION_OK front=");
          Serial.print(liftPosFront);
          Serial.print(" back=");
          Serial.println(liftPosBack);
        }
      }
      else
      {
        liftConfirmCount = 0;
      }
    }

    return true;
  }

  if (strcmp(line, "LIFT_BUSY") == 0)
  {
    liftBusy = true;
    liftAtTarget = false;
    liftHomeReached = false;
    liftResponseSequence++;
    return true;
  }

  if (sscanf(line, "LIFT_REACHED,%ld,%ld", &front, &back) == 2)
  {
    liftPosFront = (int32_t)front;
    liftPosBack = (int32_t)back;
    liftFrontCount = (int32_t)front;
    liftBackCount = (int32_t)back;
    liftResponseSequence++;

    // This reply belongs to an LP move. startLiftHome() clears liftMoveActive,
    // so a frame that was already on the wire can no longer report a homing
    // job finished the moment it started.
    if (!liftMoveActive)
    {
      Serial.print("LIFT_REACHED IGNORED (NO MOVE ACTIVE) front=");
      Serial.print(liftPosFront);
      Serial.print(" back=");
      Serial.println(liftPosBack);
      return true;
    }

    liftMoveActive = false;
    liftReached = true;
    liftBusy = false;
    liftAtTarget = true;
    liftConfirmCount = LIFT_CONFIRM_REQUIRED;
    liftTaskStatus = TASK_DONE;

    Serial.print("LIFT_POSITION_OK front=");
    Serial.print(liftPosFront);
    Serial.print(" back=");
    Serial.println(liftPosBack);
    return true;
  }

  if (strcmp(line, "LIFT_HOMING") == 0)
  {
    liftBusy = true;
    liftAtTarget = false;
    liftHomeReached = false;
    liftResponseSequence++;
    return true;
  }

  if (strcmp(line, "LIFT_HOME_REACHED") == 0)
  {
    liftResponseSequence++;

    // This reply belongs to an LZ job. RESET_STEP sends LZ and the LIFT_STEP
    // behind it sends LP about a millisecond later; without this guard the
    // homing reply lands on that LP move and completes it before the lift has
    // moved at all, which is how a LIFT_STEP used to pass without moving.
    if (!liftHomeActive)
    {
      Serial.println("LIFT_HOME_REACHED IGNORED (NO HOME ACTIVE)");
      return true;
    }

    liftHomeActive = false;
    liftBusy = false;
    liftAtTarget = true;
    liftHomeReached = true;
    liftTaskStatus = TASK_DONE;
    return true;
  }

  if (strncmp(line, "LIFT_ERROR", 10) == 0)
  {
    liftBusy = false;
    liftAtTarget = false;
    liftHomeReached = false;
    liftMoveActive = false;
    liftHomeActive = false;
    liftTaskStatus = TASK_ERROR;
    liftResponseSequence++;
    Serial.print("ERROR FROM LIFT STM32: ");
    Serial.println(line);
    return true;
  }

  return false;
}

void readLiftSerial()
{
  while (Serial7.available() > 0)
  {
    const char received = (char)Serial7.read();

    if (received == '\r' || received == '\n')
    {
      if (liftRxIndex > 0)
      {
        liftRxBuffer[liftRxIndex] = '\0';
        if (!parseLiftResponse(liftRxBuffer))
        {
          Serial.print("LIFT STM32: ");
          Serial.println(liftRxBuffer);
        }
        liftRxIndex = 0;
      }
      continue;
    }

    if (received >= 32 && received <= 126 &&
        liftRxIndex < LIFT_RX_SIZE - 1)
    {
      liftRxBuffer[liftRxIndex++] = received;
    }
    else if (liftRxIndex >= LIFT_RX_SIZE - 1)
    {
      liftRxIndex = 0;
      Serial.println("LIFT FEEDBACK TOO LONG");
    }
  }
}

// ---------- Non-blocking Lift API for mission states ----------
void startLift(int32_t front, int32_t back, uint32_t timeoutMs)
{
  liftTargetFront = front;
  liftTargetBack = back;
  liftMoveActive = true;
  liftHomeActive = false;
  liftReached = false;
  liftAtTarget = false;
  liftHomeReached = false;
  liftBusy = true;
  liftConfirmCount = 0;
  liftTaskStartMs = millis();
  liftTaskTimeoutMs = timeoutMs;
  liftTaskStatus = TASK_RUNNING;
  sendLiftCommandPulse(front, back);
}

void startLiftHome(uint32_t timeoutMs)
{
  // Homing ends at zero on both columns. Mirroring that here keeps the
  // "LIFT TIMEOUT targetF=... targetB=..." message honest when the job that
  // times out is a home rather than the LP move that ran before it.
  liftTargetFront = 0;
  liftTargetBack = 0;
  liftMoveActive = false;
  liftHomeActive = true;
  liftReached = false;
  liftAtTarget = false;
  liftHomeReached = false;
  liftBusy = true;
  liftTaskStartMs = millis();
  liftTaskTimeoutMs = timeoutMs;
  liftTaskStatus = TASK_RUNNING;
  sendLiftCommandZero();
}

bool liftDone()
{
  return liftTaskStatus == TASK_DONE;
}

bool liftRunning()
{
  return liftTaskStatus == TASK_RUNNING;
}

bool liftFailed()
{
  return liftTaskStatus == TASK_TIMEOUT ||
         liftTaskStatus == TASK_ERROR;
}

// Use these functions in a mission step while the robot base is stationary.
// Wait for a new LIFT_REACHED response generated after LP was sent.
bool waitLiftAtTarget(uint32_t responseSequenceBeforeCommand,
                      uint32_t timeoutMs)
{
  const uint32_t startMs = millis();

  while ((uint32_t)(millis() - startMs) < timeoutMs)
  {
    readHubSerial();
    readLiftSerial();
    readSlaveSerial();
    readGyro();

    const bool receivedNewResponse =
        liftResponseSequence != responseSequenceBeforeCommand;

    if (receivedNewResponse && liftAtTarget && !liftBusy)
    {
      Serial.print("LIFT MOVE DONE front=");
      Serial.print(liftFrontCount);
      Serial.print(" back=");
      Serial.println(liftBackCount);
      return true;
    }

    delay(1);
  }

  Serial.println("ERROR: LIFT MOVE TIMEOUT");
  return false;
}

bool moveLiftPulseAndWait(int32_t front, int32_t back, uint32_t timeoutMs)
{
  liftTargetFront = front;
  liftTargetBack = back;
  liftMoveActive = true;
  liftHomeActive = false;
  liftReached = false;
  liftAtTarget = false;
  liftBusy = true;
  liftConfirmCount = 0;

  sendLiftCommandPulse(front, back);

  const uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < timeoutMs)
  {
    readHubSerial();
    readLiftSerial();
    readSlaveSerial();
    readGyro();

    if (liftReached)
      return true;

    delay(1);
  }

  liftMoveActive = false;
  liftBusy = false;
  Serial.print("ERROR: LIFT POSITION TIMEOUT targetF=");
  Serial.print(liftTargetFront);
  Serial.print(" targetB=");
  Serial.print(liftTargetBack);
  Serial.print(" lastF=");
  Serial.print(liftPosFront);
  Serial.print(" lastB=");
  Serial.println(liftPosBack);
  return false;
}

bool homeLiftAndWait(uint32_t timeoutMs)
{
  const uint32_t sequenceBeforeCommand = liftResponseSequence;
  liftAtTarget = false;
  liftHomeReached = false;
  liftMoveActive = false;
  liftHomeActive = true;
  sendLiftCommandZero();

  const uint32_t startMs = millis();
  while ((uint32_t)(millis() - startMs) < timeoutMs)
  {
    readHubSerial();
    readLiftSerial();
    readSlaveSerial();
    readGyro();

    const bool receivedNewResponse =
        liftResponseSequence != sequenceBeforeCommand;
    if (receivedNewResponse && liftHomeReached && !liftBusy)
      return true;

    delay(1);
  }

  Serial.println("ERROR: LIFT HOME TIMEOUT");
  return false;
}
