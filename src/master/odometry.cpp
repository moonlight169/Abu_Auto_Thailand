#include "odometry.h"

#include <math.h>

#include "box_parking.h"
#include "gyro.h"
#include "position_control.h"
#include "robot_state.h"

static uint32_t lastPosePrintMs = 0;

void resetOdometry()
{
  positionX = 0.0f;
  positionY = 0.0f;
  encoderHeadingRad = 0.0f;

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
    previousEncoderCount[wheel] = encoderCount[wheel];

  firstFeedback = false;
}

void updateOdometry(const int32_t newCount[WHEEL_COUNT])
{
  if (firstFeedback)
  {
    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
      previousEncoderCount[wheel] = newCount[wheel];
    firstFeedback = false;
    return;
  }

  float wheelDistance[WHEEL_COUNT];
  constexpr float METERS_PER_COUNT =
      (2.0f * PI * WHEEL_RADIUS_M) / COUNTS_PER_REV;

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    const int32_t deltaCount =
        newCount[wheel] - previousEncoderCount[wheel];
    previousEncoderCount[wheel] = newCount[wheel];
    wheelDistance[wheel] = deltaCount * METERS_PER_COUNT;
  }

  const float dxBody =
      (wheelDistance[FL] + wheelDistance[FR] +
       wheelDistance[RL] + wheelDistance[RR]) * 0.25f;

  const float dyBody =
      (-wheelDistance[FL] + wheelDistance[FR] +
        wheelDistance[RL] - wheelDistance[RR]) * 0.25f;

  const float dYaw =
      (-wheelDistance[FL] + wheelDistance[FR] -
        wheelDistance[RL] + wheelDistance[RR]) /
      (4.0f * L_SUM_M);

  const float oldEncoderHeading = encoderHeadingRad;
  encoderHeadingRad = normalizeAngle(encoderHeadingRad + dYaw);

  const float middleHeading = gyroOnline
      ? gyroHeadingRad
      : normalizeAngle(oldEncoderHeading + 0.5f * dYaw);
  positionX +=
      dxBody * cosf(middleHeading) - dyBody * sinf(middleHeading);
  positionY +=
      dxBody * sinf(middleHeading) + dyBody * cosf(middleHeading);
}

void printRobotPose()
{
  const float yawRad = gyroOnline
      ? gyroHeadingRad
      : encoderHeadingRad;

  Serial.print("POSE,X=");
  Serial.print(positionX, 3);
  Serial.print(",Y=");
  Serial.print(positionY, 3);
  Serial.print(",YAW=");
  Serial.print(yawRad * RAD_TO_DEG, 2);
  Serial.print(",SOURCE=");
  Serial.println(gyroOnline ? "GYRO" : "ENC");
}

void updatePosePrint()
{
  // POSE belongs to MOVE/POSITION mode only.
  if (!posePrintEnabled || !positionControlActive || boxParkingActive)
    return;

  const uint32_t now = millis();
  if ((uint32_t)(now - lastPosePrintMs) < POSE_PRINT_PERIOD_MS)
    return;

  lastPosePrintMs = now;
  printRobotPose();
}
