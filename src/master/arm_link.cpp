#include "arm_link.h"

#include <stdio.h>
#include <string.h>

ArmTaskStatus armTaskStatus = ARM_TASK_IDLE;
ArmResetState armResetState = ARM_RESET_IDLE;

char armExpectedDone[16] = {};
char armTxPacket[80] = {};

uint32_t armCommandStartMs = 0;
bool armOnline = false;

static char armRxLine[96] = {};
static size_t armRxLength = 0;
static uint32_t armNextSequence = 1;
static uint32_t armActiveSequence = 0;
static uint32_t armLastTransmitMs = 0;
static uint32_t armLastPacketMs = 0;
static uint8_t armRetryCount = 0;
static bool armAckReceived = false;

uint32_t allocateArmSequence()
{
  const uint32_t sequence = armNextSequence++;
  if (armNextSequence == 0)
    armNextSequence = 1;
  return sequence;
}

void sendArmPacket(const char *packet)
{
  Serial6.println(packet);
  Serial.print("ARM TX: ");
  Serial.println(packet);
}

void sendArmAuxCommand(const char *command)
{
  char packet[80];
  const uint32_t sequence = allocateArmSequence();
  snprintf(packet, sizeof(packet), "CMD,%lu,%s",
           (unsigned long)sequence, command);
  sendArmPacket(packet);
}

void readArmSerial()
{
  while (Serial6.available() > 0)
  {
    const char received = (char)Serial6.read();

    if (received == '\r' || received == '\n')
    {
      if (armRxLength == 0)
        continue;

      armRxLine[armRxLength] = '\0';
      armLastPacketMs = millis();
      armOnline = true;

      Serial.print("ARM RX: ");
      Serial.println(armRxLine);

      char parseLine[sizeof(armRxLine)];
      strncpy(parseLine, armRxLine, sizeof(parseLine) - 1);
      parseLine[sizeof(parseLine) - 1] = '\0';

      char *save = nullptr;
      char *messageType = strtok_r(parseLine, ",", &save);
      char *sequenceText = strtok_r(nullptr, ",", &save);
      char *commandText = strtok_r(nullptr, ",", &save);

      if (messageType != nullptr && sequenceText != nullptr)
      {
        char *sequenceEnd = nullptr;
        const uint32_t receivedSequence =
            (uint32_t)strtoul(sequenceText, &sequenceEnd, 10);
        const bool validSequence =
            sequenceEnd != sequenceText && *sequenceEnd == '\0';
        const bool matchesActive =
            armTaskStatus == ARM_TASK_RUNNING &&
            validSequence &&
            receivedSequence == armActiveSequence;
        const bool matchesCommand =
            commandText != nullptr &&
            strcmp(commandText, armExpectedDone) == 0;

        if ((strcmp(messageType, "ACK") == 0 ||
             strcmp(messageType, "STATE") == 0) &&
            matchesActive)
        {
          armAckReceived = true;
        }
        else if (strcmp(messageType, "DONE") == 0 &&
                 matchesActive && matchesCommand)
        {
          armAckReceived = true;
          armTaskStatus = ARM_TASK_DONE;
          armTxPacket[0] = '\0';
        }
        else if ((strcmp(messageType, "ERR") == 0 ||
                  strcmp(messageType, "FAULT") == 0) &&
                 matchesActive)
        {
          armAckReceived = true;
          armTaskStatus = ARM_TASK_ERROR;
          armTxPacket[0] = '\0';
        }
      }

      armRxLength = 0;
    }
    else if (received >= 32 && received <= 126 &&
             armRxLength < sizeof(armRxLine) - 1)
    {
      armRxLine[armRxLength++] = received;
    }
    else if (armRxLength >= sizeof(armRxLine) - 1)
    {
      armRxLength = 0;
      armTaskStatus = ARM_TASK_ERROR;
      Serial.println("ARM ERROR: RX LINE TOO LONG");
    }
  }
}

void updateArmCommunication()
{
  if (armTaskStatus != ARM_TASK_RUNNING)
    return;

  const uint32_t now = millis();

  // ACK only confirms that the Slave received the command.  DONE still has
  // to arrive before this deadline, otherwise a lost/faulted Slave would leave
  // a manual arm command stuck in RUNNING forever.
  if ((uint32_t)(now - armCommandStartMs) >= ARM_DEFAULT_TIMEOUT_MS)
  {
    armTaskStatus = ARM_TASK_ERROR;
    armTxPacket[0] = '\0';
    sendArmAuxCommand("STOP");
    Serial.println("ARM ERROR: COMMAND TIMEOUT");
    return;
  }

  if (armAckReceived || armTxPacket[0] == '\0')
    return;

  if ((uint32_t)(now - armLastTransmitMs) < ARM_ACK_TIMEOUT_MS)
    return;

  if (armRetryCount >= ARM_MAX_RETRIES)
  {
    armTaskStatus = ARM_TASK_ERROR;
    armTxPacket[0] = '\0';
    Serial.println("ARM ERROR: ACK TIMEOUT");
    return;
  }

  armRetryCount++;
  armLastTransmitMs = now;
  sendArmPacket(armTxPacket);
}

void startArmCommand(const char *command)
{
  if (armTaskStatus == ARM_TASK_RUNNING)
  {
    Serial.println("ARM ERROR: MASTER BUSY");
    return;
  }

  armTaskStatus = ARM_TASK_RUNNING;
  armCommandStartMs = millis();
  armLastTransmitMs = armCommandStartMs;
  armActiveSequence = allocateArmSequence();
  armRetryCount = 0;
  armAckReceived = false;

  // Save the first CSV field so DONE must match the issued command.
  size_t expectedLength = 0;
  while (command[expectedLength] != '\0' &&
         command[expectedLength] != ',' &&
         expectedLength < sizeof(armExpectedDone) - 1)
  {
    armExpectedDone[expectedLength] = command[expectedLength];
    expectedLength++;
  }
  armExpectedDone[expectedLength] = '\0';

  snprintf(armTxPacket, sizeof(armTxPacket), "CMD,%lu,%s",
           (unsigned long)armActiveSequence, command);
  sendArmPacket(armTxPacket);
}

void startArmHome()
{
  startArmCommand("HOME");
}

void startArmBottom(float degrees)
{
  if (degrees < ARM_MIN_DEG || degrees > ARM_MAX_DEG)
  {
    armTaskStatus = ARM_TASK_ERROR;
    Serial.println("ARM ERROR: BOTTOM ANGLE OUT OF RANGE (0-180)");
    return;
  }

  char command[32];
  snprintf(command, sizeof(command), "B,%.2f", degrees);
  startArmCommand(command);
}

void startArmTop(float degrees)
{
  if (degrees < ARM_MIN_DEG || degrees > ARM_MAX_DEG)
  {
    armTaskStatus = ARM_TASK_ERROR;
    Serial.println("ARM ERROR: TOP ANGLE OUT OF RANGE (0-180)");
    return;
  }

  char command[32];
  snprintf(command, sizeof(command), "T,%.2f", degrees);
  startArmCommand(command);
}

void startArmPosition(float bottomDegrees, float topDegrees)
{
  if (bottomDegrees < ARM_MIN_DEG || bottomDegrees > ARM_MAX_DEG ||
      topDegrees < ARM_MIN_DEG || topDegrees > ARM_MAX_DEG)
  {
    armTaskStatus = ARM_TASK_ERROR;
    Serial.println("ARM ERROR: POSITION ANGLE OUT OF RANGE (0-180)");
    return;
  }

  char command[48];
  snprintf(command, sizeof(command), "POS,%.2f,%.2f",
           bottomDegrees, topDegrees);
  startArmCommand(command);
}

void startArmClearAndHome()
{
  // Cancel a manual or mission Arm command before starting the reset sequence.
  if (armTaskStatus == ARM_TASK_RUNNING)
    sendArmAuxCommand("STOP");

  armTaskStatus = ARM_TASK_IDLE;
  armExpectedDone[0] = '\0';
  armTxPacket[0] = '\0';

  armResetState = ARM_RESET_WAIT_CLEAR;
  Serial.println("ARM RESET: CLEAR START");
  startArmCommand("CLEAR");
}

void updateArmClearAndHome()
{
  switch (armResetState)
  {
    case ARM_RESET_WAIT_CLEAR:
      if (armTaskStatus == ARM_TASK_DONE)
      {
        Serial.println("ARM RESET: CLEAR DONE -> HOME START");
        armResetState = ARM_RESET_WAIT_HOME;
        startArmHome();
      }
      else if (armTaskStatus == ARM_TASK_ERROR)
      {
        armResetState = ARM_RESET_IDLE;
        Serial.println("ARM RESET ERROR: CLEAR FAILED");
      }
      break;

    case ARM_RESET_WAIT_HOME:
      if (armTaskStatus == ARM_TASK_DONE)
      {
        armResetState = ARM_RESET_IDLE;
        Serial.println("ARM RESET: HOME COMPLETE");
      }
      else if (armTaskStatus == ARM_TASK_ERROR)
      {
        armResetState = ARM_RESET_IDLE;
        Serial.println("ARM RESET ERROR: HOME FAILED");
      }
      break;

    default:
      break;
  }
}
