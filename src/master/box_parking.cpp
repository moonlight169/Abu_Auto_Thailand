#include "box_parking.h"

#include <math.h>

#include "box_result.h"
#include "hub_link.h"
#include "kinematics.h"
#include "position_control.h"
#include "robot_state.h"
#include "tasks.h"
#include "wheel_link.h"

bool boxParkingActive = false;
bool wallAlignmentMode = false;
bool hubDetectionIsWall = false;
float boxParkingTargetMm = 450.0f;

float parkingVx = 0.0f;
float parkingVy = 0.0f;
float parkingWz = 0.0f;

uint32_t boxParkingInToleranceSinceMs = 0;

static uint32_t boxParkingStartMs = 0;
static uint32_t boxParkingTimeoutMs = 0;
static uint32_t lastCheckedBoxFrame = 0;
static uint32_t lastLidarControlMs = 0;
static uint32_t lastLidarPrintMs = 0;
static uint32_t lastBoxNotFoundPrintMs = 0;

static float applyParkingMinimum(float value, float minimumValue)
{
  if (value > 0.0f && value < minimumValue)
    return minimumValue;
  if (value < 0.0f && value > -minimumValue)
    return -minimumValue;
  return value;
}

static float limitParkingChange(float target, float current, float maxChange)
{
  return constrain(target, current - maxChange, current + maxChange);
}

static void resetParkingVelocity()
{
  parkingVx = 0.0f;
  parkingVy = 0.0f;
  parkingWz = 0.0f;
}

void startBoxParking(float targetDistanceMm, uint32_t timeoutMs,
                     bool alignWall)
{
  wallAlignmentMode = alignWall;
  positionControlActive = false;
  velocityMode = VELOCITY_STOPPED;
  commandedVx = commandedVy = commandedWz = 0.0f;
  boxParkingTargetMm = max(targetDistanceMm, 150.0f);
  boxParkingStartMs = millis();
  boxParkingTimeoutMs = timeoutMs;
  boxParkingInToleranceSinceMs = 0;
  lastCheckedBoxFrame = boxFrameCounter;
  lostBoxScans = 0;
  lastBoxSeenMs = 0;
  clearBoxResult();
  lidarLastScanPointCount = 0;
  lastLidarPrintMs = 0;
  lastBoxNotFoundPrintMs = 0;
  resetParkingVelocity();
  boxParkingTaskStatus = TASK_RUNNING;
  boxParkingActive = true;
  commandActive = true;

  // Ask the Hub to process LiDAR in box-detection mode.  LiDAR is connected
  // to the Hub; the Master receives only the processed HUB packet on Serial1.
  Serial1.println(wallAlignmentMode ? "MODE WALL" : "MODE BOX");
  Serial1.println("LIDAR ON");

  Serial.print(wallAlignmentMode
                   ? "HUB WALL ALIGN START target="
                   : "HUB LIDAR PARK START target=");
  Serial.print(boxParkingTargetMm, 0);
  Serial.println(" mm");
}

void startWallAlignment(float targetDistanceMm, uint32_t timeoutMs)
{
  // Wall mode returns distance and wall angle. Its lateral offset is not used,
  // because a long wall has no meaningful center like a box does.
  startBoxParking(targetDistanceMm, timeoutMs, true);
}

bool boxParkingDone()
{
  return boxParkingTaskStatus == TASK_DONE;
}

bool boxParkingFailed()
{
  return boxParkingTaskStatus == TASK_TIMEOUT ||
         boxParkingTaskStatus == TASK_ERROR;
}

void updateBoxParking()
{
  if (!boxParkingActive)
    return;

  const uint32_t now = millis();
  if ((uint32_t)(now - lastLidarControlMs) < LIDAR_CONTROL_PERIOD_MS)
    return;
  lastLidarControlMs = now;

  if (boxParkingTimeoutMs > 0 &&
      (uint32_t)(now - boxParkingStartMs) >= boxParkingTimeoutMs)
  {
    boxParkingActive = false;
    boxParkingTaskStatus = TASK_TIMEOUT;
    stopRobot();
    Serial.println("ERROR: LIDAR PARK TIMEOUT");
    return;
  }

  if (!hub.online ||
      hubDetectionIsWall != wallAlignmentMode ||
      !boxResult.found ||
      lastBoxSeenMs == 0 ||
      (uint32_t)(now - lastBoxSeenMs) > HUB_TIMEOUT_MS)
  {
    boxParkingInToleranceSinceMs = 0;
    resetParkingVelocity();
    setLocalVelocity(0.0f, 0.0f, 0.0f);
    sendTargetPacket();

    if ((lastBoxSeenMs == 0 ||
         (uint32_t)(now - lastBoxSeenMs) > LIDAR_LOST_TIMEOUT_MS) &&
        (uint32_t)(now - lastBoxNotFoundPrintMs) >= 500)
    {
      lastBoxNotFoundPrintMs = now;
      if (!hub.online)
        Serial.println("HUB LIDAR: HUB OFFLINE");
      else if (hubDetectionIsWall != wallAlignmentMode)
        Serial.println(wallAlignmentMode
                           ? "HUB LIDAR: WAITING FOR WALL MODE"
                           : "HUB LIDAR: WAITING FOR BOX MODE");
      else
        Serial.println(wallAlignmentMode
                           ? "HUB WALL ALIGN: WALL NOT FOUND"
                           : "HUB LIDAR PARK: BOX NOT FOUND");
    }
    return;
  }

  const float distanceError =
      boxResult.distanceMm - boxParkingTargetMm;
  const float offsetError = boxResult.offsetMm;
  const float angleError = boxResult.angleDeg;

  const bool distanceOk =
      fabsf(distanceError) <= PARK_DISTANCE_TOL_MM;
  const bool offsetOk =
      wallAlignmentMode ||
      fabsf(offsetError) <= PARK_OFFSET_TOL_MM;
  const bool angleOk =
      fabsf(angleError) <= PARK_ANGLE_TOL_DEG;

  const bool newBoxFrame = boxFrameCounter != lastCheckedBoxFrame;
  if (newBoxFrame)
    lastCheckedBoxFrame = boxFrameCounter;

  if (distanceOk && offsetOk && angleOk)
  {
    resetParkingVelocity();
    setLocalVelocity(0.0f, 0.0f, 0.0f);
    sendTargetPacket();

    if (newBoxFrame && boxParkingInToleranceSinceMs == 0)
      boxParkingInToleranceSinceMs = now;

    if ((uint32_t)(now - boxParkingInToleranceSinceMs) >=
        PARK_DONE_HOLD_MS)
    {
      boxParkingActive = false;
      boxParkingTaskStatus = TASK_DONE;
      stopRobot();
      Serial.print(wallAlignmentMode
                       ? "HUB WALL ALIGN DONE distance="
                       : "HUB LIDAR PARK DONE distance=");
      Serial.print(boxResult.distanceMm, 0);
      Serial.print(" offset=");
      Serial.print(boxResult.offsetMm, 0);
      Serial.print(" angle=");
      Serial.println(boxResult.angleDeg, 2);
    }
    return;
  }

  boxParkingInToleranceSinceMs = 0;

  float targetVx = 0.0f;
  float targetVy = 0.0f;
  float targetWz = 0.0f;

  if (fabsf(distanceError) > PARK_DISTANCE_DEADBAND_MM)
  {
    const float limit = fabsf(distanceError) > 100.0f
                            ? PARK_MAX_VX
                            : 0.045f;
    targetVx = constrain(PARK_KP_DISTANCE * distanceError,
                         -limit, limit);
    targetVx = applyParkingMinimum(targetVx, PARK_MIN_VX);
  }
  if (!wallAlignmentMode &&
      fabsf(offsetError) > PARK_OFFSET_DEADBAND_MM)
  {
    const float limit = fabsf(offsetError) > 80.0f
                            ? PARK_MAX_VY
                            : 0.035f;
    targetVy = constrain(PARK_KP_OFFSET * offsetError,
                         -limit, limit);
    targetVy = applyParkingMinimum(targetVy, PARK_MIN_VY);
  }
  if (fabsf(angleError) > PARK_ANGLE_DEADBAND_DEG)
  {
    const float limit = fabsf(angleError) > 5.0f
                            ? PARK_MAX_WZ
                            : 0.050f;
    targetWz = constrain(PARK_KP_ANGLE * angleError,
                         -limit, limit);
    targetWz = applyParkingMinimum(targetWz, PARK_MIN_WZ);
  }

  if (fabsf(angleError) > 5.0f)
  {
    targetVx *= 0.50f;
    targetVy *= 0.50f;
  }

  parkingVx =
      limitParkingChange(targetVx, parkingVx, PARK_ACCEL_VX);
  parkingVy =
      limitParkingChange(targetVy, parkingVy, PARK_ACCEL_VY);
  parkingWz =
      limitParkingChange(targetWz, parkingWz, PARK_ACCEL_WZ);

  setLocalVelocity(
      PARK_VX_SIGN * parkingVx,
      PARK_VY_SIGN * -parkingVy,
      PARK_WZ_SIGN * parkingWz);
  sendTargetPacket();
}

static void printLidarBoxLine(float offsetMm, float widthMm)
{
  constexpr int BAR_WIDTH = 41;
  constexpr int CENTER_INDEX = BAR_WIDTH / 2;
  constexpr float DISPLAY_HALF_WIDTH_MM = 1000.0f;
  char bar[BAR_WIDTH + 1];

  for (int i = 0; i < BAR_WIDTH; i++)
    bar[i] = '-';
  bar[BAR_WIDTH] = '\0';
  bar[CENTER_INDEX] = '|';

  const float boxLeftMm = offsetMm - widthMm * 0.5f;
  const float boxRightMm = offsetMm + widthMm * 0.5f;
  int leftIndex = CENTER_INDEX +
      (int)(boxLeftMm / DISPLAY_HALF_WIDTH_MM * CENTER_INDEX);
  int rightIndex = CENTER_INDEX +
      (int)(boxRightMm / DISPLAY_HALF_WIDTH_MM * CENTER_INDEX);
  leftIndex = constrain(leftIndex, 0, BAR_WIDTH - 1);
  rightIndex = constrain(rightIndex, 0, BAR_WIDTH - 1);

  if (leftIndex > rightIndex)
  {
    const int temp = leftIndex;
    leftIndex = rightIndex;
    rightIndex = temp;
  }

  for (int i = leftIndex; i <= rightIndex; i++)
  {
    if (i != CENTER_INDEX)
      bar[i] = 'o';
  }

  Serial.print("BOX  [");
  Serial.print(bar);
  Serial.println("]");
}

void updateLidarPrint()
{
  // LiDAR diagnostics belong to LIDAR PARK mode only.
  if (!lidarPrintEnabled || !boxParkingActive)
    return;

  const uint32_t now = millis();
  if ((uint32_t)(now - lastLidarPrintMs) < LIDAR_PRINT_PERIOD_MS)
    return;

  lastLidarPrintMs = now;

  Serial.print("HUB_LIDAR,packetAge=");
  if (hub.lastPacketMs == 0)
    Serial.print("NONE");
  else
    Serial.print((uint32_t)(now - hub.lastPacketMs));

  Serial.print(",hubPackets=");
  Serial.print(hub.goodPackets);
  Serial.print(",frames=");
  Serial.print(boxFrameCounter);
  Serial.print(",points=");
  Serial.print(lidarLastScanPointCount);

  if (boxResult.found &&
      lastBoxSeenMs != 0 &&
      (uint32_t)(now - lastBoxSeenMs) <= 300)
  {
    Serial.print(",BOX,distance=");
    Serial.print(boxResult.distanceMm, 0);
    Serial.print(",offset=");
    Serial.print(boxResult.offsetMm, 0);
    Serial.print(",angle=");
    Serial.print(boxResult.angleDeg, 2);
    Serial.print(",width=");
    Serial.print(boxResult.widthMm, 0);
    Serial.print(",lineError=");
    Serial.print(boxResult.lineErrorMm, 1);
    Serial.print(",cmdVx=");
    Serial.print(parkingVx, 3);
    Serial.print(",cmdVy=");
    Serial.print(parkingVy, 3);
    Serial.print(",cmdWz=");
    Serial.println(parkingWz, 3);
    printLidarBoxLine(boxResult.offsetMm, boxResult.widthMm);
  }
  else
  {
    Serial.print(",NO_BOX,lost=");
    Serial.println(lostBoxScans);
  }
}
