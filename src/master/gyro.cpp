#include "gyro.h"

#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <math.h>

#include "robot_state.h"

static Adafruit_BNO08x bno08x(BNO08X_RESET);
static sh2_SensorValue_t bnoSensorValue;

float gyroHeadingRad = 0.0f;
float gyroYawDeg = 0.0f;
float gyroRollDeg = 0.0f;
float gyroPitchDeg = 0.0f;
float gyroRawYawDeg = 0.0f;
float gyroYawOffsetDeg = 0.0f;

bool gyroOnline = false;
uint32_t lastGyroMs = 0;

static bool enableGyroReport()
{
  return bno08x.enableReport(
      SH2_GAME_ROTATION_VECTOR,
      GYRO_REPORT_INTERVAL_US);
}

bool beginGyro()
{
  Wire.begin();  // Teensy 4.1: SDA pin 18, SCL pin 19
  Wire.setClock(400000);

  bool found = bno08x.begin_I2C(0x4A, &Wire);
  if (!found)
    found = bno08x.begin_I2C(0x4B, &Wire);

  if (!found)
  {
    Serial.println("ERROR: BNO085 NOT FOUND (check SDA18/SCL19)");
    gyroOnline = false;
    return false;
  }

  if (!enableGyroReport())
  {
    Serial.println("ERROR: CANNOT ENABLE BNO085 REPORT");
    gyroOnline = false;
    return false;
  }

  Serial.println("BNO085 READY ON SDA18/SCL19");
  return true;
}

void readGyro()
{
  if (bno08x.wasReset())
  {
    gyroOnline = false;
    if (!enableGyroReport())
    {
      Serial.println("WARNING: BNO085 REPORT RESTART FAILED");
      return;
    }
  }

  while (bno08x.getSensorEvent(&bnoSensorValue))
  {
    if (bnoSensorValue.sensorId != SH2_GAME_ROTATION_VECTOR)
      continue;

    const float qr = bnoSensorValue.un.gameRotationVector.real;
    const float qi = bnoSensorValue.un.gameRotationVector.i;
    const float qj = bnoSensorValue.un.gameRotationVector.j;
    const float qk = bnoSensorValue.un.gameRotationVector.k;

    const float rollRad = atan2f(
        2.0f * (qr * qi + qj * qk),
        1.0f - 2.0f * (qi * qi + qj * qj));

    float sinPitch = 2.0f * (qr * qj - qk * qi);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    const float pitchRad = asinf(sinPitch);

    const float yawRad = atan2f(
        2.0f * (qr * qk + qi * qj),
        1.0f - 2.0f * (qj * qj + qk * qk));

    gyroRollDeg = rollRad * RAD_TO_DEG;
    gyroPitchDeg = pitchRad * RAD_TO_DEG;
    gyroRawYawDeg = yawRad * RAD_TO_DEG;
    gyroYawDeg = normalizeAngleDeg(gyroRawYawDeg - gyroYawOffsetDeg);
    gyroHeadingRad = gyroYawDeg * DEG_TO_RAD;
    gyroOnline = true;
    lastGyroMs = millis();
  }
}

void resetGyroYaw()
{
  gyroYawOffsetDeg = gyroRawYawDeg;
  gyroYawDeg = 0.0f;
  gyroHeadingRad = 0.0f;
  Serial.println("BNO085 YAW RESET TO 0 DEG");
}

void printSensorStatus()
{
  Serial.print("GYRO=");
  Serial.print(gyroOnline ? "ONLINE" : "OFFLINE");
  Serial.print(" roll=");
  Serial.print(gyroRollDeg, 2);
  Serial.print(" pitch=");
  Serial.print(gyroPitchDeg, 2);
  Serial.print(" yaw=");
  Serial.println(gyroYawDeg, 2);
}
