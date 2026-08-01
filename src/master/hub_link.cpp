#include "hub_link.h"

#include <stdio.h>
#include <string.h>

#include "box_parking.h"
#include "box_result.h"

HubState hub = {};

static char hubRxBuffer[HUB_RX_SIZE];
static size_t hubRxIndex = 0;

// Extra hardware RX memory for the high-rate HUB stream on Teensy 4.1.
// Serial1.addMemoryForRead() must be called before Serial1.begin().
static uint8_t hubSerialRxMemory[HUB_SERIAL_RX_MEMORY_SIZE];

void beginHubSerial()
{
  Serial1.addMemoryForRead(hubSerialRxMemory,
                           sizeof(hubSerialRxMemory));
  Serial1.begin(HUB_BAUD);
}

bool hubInputActive(uint8_t bit)
{
  return hub.online && bit < 16 &&
         (hub.inputMask & (1u << bit)) != 0;
}

bool tfDistanceInRange(uint8_t tfIndex,
                       uint16_t minDistanceCm,
                       uint16_t maxDistanceCm)
{
  if (!hub.online || tfIndex >= 4 ||
      (hub.tfValidMask & (1u << tfIndex)) == 0)
    return false;

  const uint16_t distanceCm = hub.tfDistanceCm[tfIndex];
  return distanceCm >= minDistanceCm &&
         distanceCm <= maxDistanceCm;
}

bool hubLaserDetected(uint8_t laserBit)
{
  switch (laserBit)
  {
    case HUB_LASER4:
      return tfDistanceInRange(0, TF1_MIN_CM, TF1_MAX_CM);

    case HUB_LASER3:
      return tfDistanceInRange(1, TF2_MIN_CM, TF2_MAX_CM);

    case HUB_LASER2:
      return tfDistanceInRange(2, TF3_MIN_CM, TF3_MAX_CM);

    default:
      return hubInputActive(laserBit);
  }
}

bool tf2ForwardStopDetected()
{
  constexpr uint8_t TF2_INDEX = 1;

  if (!hub.online ||
      (hub.tfValidMask & (1u << TF2_INDEX)) == 0)
  {
    return false;
  }

  const uint16_t tf2DistanceCm = hub.tfDistanceCm[TF2_INDEX];
  return tf2DistanceCm > 0 &&
         tf2DistanceCm < TF2_FORWARD_STOP_CM;
}

bool tf1ForwardStopDetected()
{
  constexpr uint8_t TF1_INDEX = 0;

  if (!hub.online ||
      (hub.tfValidMask & (1u << TF1_INDEX)) == 0)
  {
    return false;
  }

  const uint16_t tf1DistanceCm = hub.tfDistanceCm[TF1_INDEX];
  return tf1DistanceCm > 0 &&
         tf1DistanceCm < TF1_FORWARD_STOP_CM;
}

bool tf1DescendFloorDetected()
{
  constexpr uint8_t TF1_INDEX = 0;

  if (!hub.online ||
      (hub.tfValidMask & (1u << TF1_INDEX)) == 0)
  {
    return false;
  }

  const uint16_t tf1DistanceCm = hub.tfDistanceCm[TF1_INDEX];
  return tf1DistanceCm > 0 &&
         tf1DistanceCm <= TF1_DESCEND_FLOOR_MAX_CM;
}

void setHubRelay(uint8_t relayNumber, bool on)
{
  if (relayNumber < 1 || relayNumber > 4)
    return;
  Serial1.print('R');
  Serial1.print(relayNumber);
  Serial1.println(on ? " ON" : " OFF");
}

void setHubArm(uint8_t angle)
{
  Serial1.print("ARM ");
  Serial1.println(min(angle, HUB_ARM_MAX_DEG));
}

void setHubSpin(uint8_t angle)
{
  Serial1.print("SPIN ");
  Serial1.println(min(angle, HUB_SPIN_MAX_DEG));
}

void printHubStatus()
{
  Serial.print("HUB=");
  Serial.print(hub.online ? "ONLINE" : "OFFLINE");
  Serial.print(",inputMask=");
  Serial.print(hub.inputMask);
  Serial.print(",LDR1=");
  Serial.print(hubInputActive(HUB_LDR1) ? "PRESSED" : "RELEASED");
  Serial.print(",LDR2=");
  Serial.print(hubInputActive(HUB_LDR2) ? "PRESSED" : "RELEASED");
  Serial.print(",Laser5=");
  Serial.print(hubLaserDetected(HUB_LASER5) ? "DETECTED" : "CLEAR");
  Serial.print(",relayMask=");
  Serial.print(hub.relayMask);
  Serial.print(",arm=");
  Serial.print(hub.armAngle);
  Serial.print(",spin=");
  Serial.print(hub.spinAngle);
  Serial.print(",TF1=");
  Serial.print(hub.tfDistanceCm[0]);
  Serial.print(hub.tfValidMask & 0x01 ? "cm(valid)" : "cm(invalid)");
  Serial.print(",TF2=");
  Serial.print(hub.tfDistanceCm[1]);
  Serial.print(hub.tfValidMask & 0x02 ? "cm(valid)" : "cm(invalid)");
  Serial.print(",TF3=");
  Serial.print(hub.tfDistanceCm[2]);
  Serial.print(hub.tfValidMask & 0x04 ? "cm(valid)" : "cm(invalid)");
  Serial.print(",TF4=");
  Serial.print(hub.tfDistanceCm[3]);
  Serial.print(hub.tfValidMask & 0x08 ? "cm(valid)" : "cm(invalid)");
  Serial.print(",lidarFound=");
  Serial.print(boxResult.found ? 1 : 0);
  Serial.print(",distance=");
  Serial.print(boxResult.distanceMm, 0);
  Serial.print(",offset=");
  Serial.print(boxResult.offsetMm, 0);
  Serial.print(",angle=");
  Serial.print(boxResult.angleDeg, 2);
  Serial.print(",good=");
  Serial.print(hub.goodPackets);
  Serial.print(",bad=");
  Serial.println(hub.badPackets);
}

bool parseHubPacket(char *line)
{
  unsigned long slaveTime = 0;
  unsigned int inputMask = 0;
  unsigned int relayMask = 0;
  unsigned int arm = 0;
  unsigned int spin = 0;
  unsigned int tf1 = 0;
  unsigned int tf2 = 0;
  unsigned int tf3 = 0;
  unsigned int tf4 = 0;
  unsigned int tfValidMask = 0;

  if (strncmp(line, "HUB,", 4) == 0)
  {
    char *savePtr = nullptr;
    char *token[18] = {};
    uint8_t tokenCount = 0;
    char *part = strtok_r(line, ",", &savePtr);
    while (part != nullptr && tokenCount < 18)
    {
      token[tokenCount++] = part;
      part = strtok_r(nullptr, ",", &savePtr);
    }

    if ((tokenCount == 13 || tokenCount == 17 || tokenCount == 18) &&
        strcmp(token[0], "HUB") == 0)
    {
      slaveTime = strtoul(token[1], nullptr, 10);
      hubDetectionIsWall = strcmp(token[2], "WALL") == 0;
      const int found = atoi(token[3]);
      const float distance = strtof(token[4], nullptr);
      const float offset = strtof(token[5], nullptr);
      const float angle = strtof(token[6], nullptr);
      const float width = strtof(token[7], nullptr);
      const unsigned long points = strtoul(token[8], nullptr, 10);
      inputMask = (unsigned int)strtoul(token[9], nullptr, 10);
      relayMask = (unsigned int)strtoul(token[10], nullptr, 10);
      arm = (unsigned int)strtoul(token[11], nullptr, 10);
      spin = (unsigned int)strtoul(token[12], nullptr, 10);
      if (tokenCount == 17 || tokenCount == 18)
      {
        tf1 = (unsigned int)strtoul(token[13], nullptr, 10);
        tf2 = (unsigned int)strtoul(token[14], nullptr, 10);
        tf3 = (unsigned int)strtoul(token[15], nullptr, 10);
        if (tokenCount == 18)
        {
          tf4 = (unsigned int)strtoul(token[16], nullptr, 10);
          tfValidMask = (unsigned int)strtoul(token[17], nullptr, 10);
        }
        else
        {
          tfValidMask = (unsigned int)strtoul(token[16], nullptr, 10);
        }
      }
      else
      {
        // Legacy packet has no TFMini-S data.
        hub.tfValidMask = 0;
      }

      // LIDAR_PARK uses the box result calculated by the Hub.
      boxFrameCounter++;
      boxResult.found = found != 0;
      boxResult.distanceMm = distance;
      boxResult.offsetMm = offset;
      boxResult.angleDeg = angle;
      boxResult.widthMm = width;
      boxResult.lineErrorMm = 0.0f;
      boxResult.pointCount = (uint16_t)min(points, 65535ul);
      lidarLastScanPointCount = boxResult.pointCount;

      if (boxResult.found)
        lastBoxSeenMs = millis();

      hub.armAngle = (uint8_t)min(arm, 255u);
      hub.spinAngle = (uint8_t)min(spin, 255u);
      hub.tfDistanceCm[0] = (uint16_t)min(tf1, 65535u);
      hub.tfDistanceCm[1] = (uint16_t)min(tf2, 65535u);
      hub.tfDistanceCm[2] = (uint16_t)min(tf3, 65535u);
      hub.tfDistanceCm[3] = (uint16_t)min(tf4, 65535u);
      hub.tfValidMask = (uint8_t)(tfValidMask & 0x0Fu);
    }
    else
    {
      // Short format:
      // HUB,time,inputMask,relayMask[,arm,spin[,tf1,tf2,tf3[,tf4],tfValidMask]]
      if (tokenCount < 4 ||
          (tokenCount > 6 && tokenCount != 10 && tokenCount != 11) ||
          strcmp(token[0], "HUB") != 0)
        return false;
      slaveTime = strtoul(token[1], nullptr, 10);
      inputMask = (unsigned int)strtoul(token[2], nullptr, 10);
      relayMask = (unsigned int)strtoul(token[3], nullptr, 10);
      if (tokenCount >= 5)
      {
        arm = (unsigned int)strtoul(token[4], nullptr, 10);
        hub.armAngle = (uint8_t)min(arm, 255u);
      }
      if (tokenCount >= 6)
      {
        spin = (unsigned int)strtoul(token[5], nullptr, 10);
        hub.spinAngle = (uint8_t)min(spin, 255u);
      }
      if (tokenCount == 10 || tokenCount == 11)
      {
        tf1 = (unsigned int)strtoul(token[6], nullptr, 10);
        tf2 = (unsigned int)strtoul(token[7], nullptr, 10);
        tf3 = (unsigned int)strtoul(token[8], nullptr, 10);
        if (tokenCount == 11)
        {
          tf4 = (unsigned int)strtoul(token[9], nullptr, 10);
          tfValidMask = (unsigned int)strtoul(token[10], nullptr, 10);
        }
        else
        {
          tfValidMask = (unsigned int)strtoul(token[9], nullptr, 10);
        }
        hub.tfDistanceCm[0] = (uint16_t)min(tf1, 65535u);
        hub.tfDistanceCm[1] = (uint16_t)min(tf2, 65535u);
        hub.tfDistanceCm[2] = (uint16_t)min(tf3, 65535u);
        hub.tfDistanceCm[3] = (uint16_t)min(tf4, 65535u);
        hub.tfValidMask = (uint8_t)(tfValidMask & 0x0Fu);
      }
      else
      {
        hub.tfValidMask = 0;
      }
    }
  }
  else
    return false;

  hub.slaveTimeMs = (uint32_t)slaveTime;
  hub.inputMask = (uint16_t)inputMask;
  hub.relayMask = (uint8_t)(relayMask & 0x0Fu);
  hub.lastPacketMs = millis();
  hub.online = true;
  hub.goodPackets++;
  return true;
}

void readHubSerial()
{
  while (Serial1.available() > 0)
  {
    const char received = (char)Serial1.read();
    if (received == '\r' || received == '\n')
    {
      if (hubRxIndex > 0)
      {
        hubRxBuffer[hubRxIndex] = '\0';
        if (strncmp(hubRxBuffer, "HUB,", 4) == 0)
        {
          if (!parseHubPacket(hubRxBuffer))
          {
            hub.badPackets++;
            Serial.print("BAD HUB PACKET: ");
            Serial.println(hubRxBuffer);
          }
        }
        else
        {
          Serial.print("HUB MSG: ");
          Serial.println(hubRxBuffer);
        }
        hubRxIndex = 0;
      }
      continue;
    }

    if (received >= 32 && received <= 126 &&
        hubRxIndex < HUB_RX_SIZE - 1)
    {
      hubRxBuffer[hubRxIndex++] = received;

      // Recover immediately if a newline was lost and a new HUB frame became
      // attached to the damaged frame, for example "...15HUB,123,...".
      if (hubRxIndex > 4 &&
          hubRxBuffer[hubRxIndex - 4] == 'H' &&
          hubRxBuffer[hubRxIndex - 3] == 'U' &&
          hubRxBuffer[hubRxIndex - 2] == 'B' &&
          hubRxBuffer[hubRxIndex - 1] == ',')
      {
        hubRxBuffer[0] = 'H';
        hubRxBuffer[1] = 'U';
        hubRxBuffer[2] = 'B';
        hubRxBuffer[3] = ',';
        hubRxIndex = 4;
        hub.badPackets++;
        Serial.println("HUB FRAME RESYNC");
      }
    }
    else if (hubRxIndex >= HUB_RX_SIZE - 1)
    {
      hubRxIndex = 0;
      hub.badPackets++;
      Serial.println("HUB RX BUFFER OVERFLOW");
    }
  }
}
