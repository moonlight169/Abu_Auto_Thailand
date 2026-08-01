#include "wheel_link.h"

#include <stdio.h>
#include <string.h>

#include "gyro.h"
#include "odometry.h"
#include "robot_state.h"

uint32_t lastCommandSendMs = 0;
uint32_t lastFeedbackMs = 0;
uint32_t goodPacketCount = 0;
uint32_t badPacketCount = 0;

static char slaveRxBuffer[SLAVE_RX_SIZE];
static size_t slaveRxIndex = 0;

void sendTargetPacket()
{
  Serial2.print("T,");
  Serial2.print((int)lroundf(targetRPM[FL]));
  Serial2.print(',');
  Serial2.print((int)lroundf(targetRPM[FR]));
  Serial2.print(',');
  Serial2.print((int)lroundf(targetRPM[RL]));
  Serial2.print(',');
  Serial2.println((int)lroundf(targetRPM[RR]));
  lastCommandSendMs = millis();
}

void printFeedback(uint32_t slaveTime)
{
  Serial.print("F t=");
  Serial.print(slaveTime);
  Serial.print(" C=");

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    Serial.print(encoderCount[wheel]);
    if (wheel < WHEEL_COUNT - 1)
      Serial.print(',');
  }

  Serial.print(" RPM=");
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    Serial.print(actualRPM[wheel], 1);
    if (wheel < WHEEL_COUNT - 1)
      Serial.print(',');
  }

  Serial.print(" POS x=");
  Serial.print(positionX, 3);
  Serial.print(" y=");
  Serial.print(positionY, 3);
  Serial.print(" yaw=");
  if (gyroOnline)
    Serial.print(gyroYawDeg, 1);
  else
    Serial.print(encoderHeadingRad * 180.0f / PI, 1);
  Serial.print(gyroOnline ? " deg(GYRO)" : " deg(ENC)");
  Serial.print(" encYaw=");
  Serial.print(encoderHeadingRad * 180.0f / PI, 1);
  Serial.println(" deg");
}

static bool parseFeedback(char *line)
{
  unsigned long slaveTime = 0;
  long countFL = 0;
  long countFR = 0;
  long countRL = 0;
  long countRR = 0;
  float rpmFL = 0.0f;
  float rpmFR = 0.0f;
  float rpmRL = 0.0f;
  float rpmRR = 0.0f;

  const int fields = sscanf(
      line,
      "F,%lu,%ld,%ld,%ld,%ld,%f,%f,%f,%f",
      &slaveTime,
      &countFL, &countFR, &countRL, &countRR,
      &rpmFL, &rpmFR, &rpmRL, &rpmRR);

  if (fields != 9)
    return false;

  const int32_t newCount[WHEEL_COUNT] = {
      (int32_t)countFL, (int32_t)countFR,
      (int32_t)countRL, (int32_t)countRR};

  actualRPM[FL] = rpmFL;
  actualRPM[FR] = rpmFR;
  actualRPM[RL] = rpmRL;
  actualRPM[RR] = rpmRR;

  updateOdometry(newCount);

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
    encoderCount[wheel] = newCount[wheel];

  lastFeedbackMs = millis();
  feedbackOnline = true;
  goodPacketCount++;
  // printFeedback((uint32_t)slaveTime);
  return true;
}

static void processSlaveLine(char *line)
{
  if (line[0] == 'F' && line[1] == ',')
  {
    if (!parseFeedback(line))
    {
      badPacketCount++;
      Serial.print("BAD FEEDBACK: ");
      Serial.println(line);
    }
    return;
  }

  Serial.print("STM32: ");
  Serial.println(line);
}

void readSlaveSerial()
{
  while (Serial2.available() > 0)
  {
    const char received = (char)Serial2.read();

    if (received == '\r' || received == '\n')
    {
      if (slaveRxIndex > 0)
      {
        slaveRxBuffer[slaveRxIndex] = '\0';
        processSlaveLine(slaveRxBuffer);
        slaveRxIndex = 0;
      }
      continue;
    }

    if (slaveRxIndex < SLAVE_RX_SIZE - 1)
    {
      slaveRxBuffer[slaveRxIndex++] = received;
    }
    else
    {
      slaveRxIndex = 0;
      badPacketCount++;
      Serial.println("STM32 RX BUFFER OVERFLOW");
    }
  }
}

void printStatus()
{
  Serial.print("LINK=");
  Serial.print(feedbackOnline ? "ONLINE" : "OFFLINE");
  Serial.print(" TARGET=");

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    Serial.print(targetRPM[wheel], 1);
    if (wheel < WHEEL_COUNT - 1)
      Serial.print(',');
  }

  Serial.print(" PACKET good=");
  Serial.print(goodPacketCount);
  Serial.print(" bad=");
  Serial.print(badPacketCount);
  Serial.print(" GYRO=");
  Serial.print(gyroOnline ? "ONLINE" : "OFFLINE");
  Serial.print(" yaw=");
  Serial.println(gyroYawDeg, 2);
}
