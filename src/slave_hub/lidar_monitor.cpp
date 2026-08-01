#include "lidar_monitor.h"

#include "lidar.h"

// Start with USB LiDAR reporting disabled. Use "LIDAR ON" to test.
uint8_t MONITOR_MODE = 0;
uint32_t MONITOR_INTERVAL_MS = 50;

static uint32_t lastMonitorTime = 0;

static void printBoxBar(float offsetMm, float widthMm) {
  char bar[BOX_BAR_WIDTH + 1];
  for (int i = 0; i < BOX_BAR_WIDTH; i++) {
    bar[i] = '-';
  }
  bar[BOX_BAR_WIDTH] = '\0';

  const int center = BOX_BAR_WIDTH / 2;
  const float boxLeftMm = offsetMm - widthMm * 0.5f;
  const float boxRightMm = offsetMm + widthMm * 0.5f;

  int leftIndex = center +
      (int)(boxLeftMm / BOX_BAR_HALF_WIDTH_MM * center);
  int rightIndex = center +
      (int)(boxRightMm / BOX_BAR_HALF_WIDTH_MM * center);
  leftIndex = constrain(leftIndex, 0, BOX_BAR_WIDTH - 1);
  rightIndex = constrain(rightIndex, 0, BOX_BAR_WIDTH - 1);

  if (leftIndex > rightIndex) {
    const int temporary = leftIndex;
    leftIndex = rightIndex;
    rightIndex = temporary;
  }

  for (int i = leftIndex; i <= rightIndex; i++) {
    bar[i] = 'o';
  }
  bar[center] = '|';

  Serial.print("BOX  [");
  Serial.print(bar);
  Serial.println("]");
}

void printMonitorSummary() {
  Serial.print("LIDAR,bytesAge=");
  if (lastPacketTime == 0) {
    Serial.print("NONE");
  } else {
    Serial.print(millis() - lastPacketTime);
  }
  Serial.print(",validPackets=");
  Serial.print(validPacketCount);
  Serial.print(",scans=");
  Serial.print(scanFrameCount);
  Serial.print(",points=");
  Serial.print(monitorPointCount);
  Serial.print(",clusters=");
  Serial.print(lastClusterCount);
  Serial.print(",lines=");
  Serial.print(lastLineCount);

  const bool detectionFresh =
      result.found &&
      lastDetectionFrameTime != 0 &&
      millis() - lastDetectionFrameTime <= 300;

  Serial.print(",mode=");
  Serial.print(detectionMode == MODE_BOX ? "BOX" : "WALL");

  if (!detectionFresh) {
    Serial.print(detectionMode == MODE_BOX ? ",NO_BOX,lost="
                                          : ",NO_WALL,lost=");
    Serial.println(lostCount);
    return;
  }

  Serial.print(detectionMode == MODE_BOX ? ",BOX,distance="
                                        : ",WALL,distance=");
  Serial.print(result.distanceMm, 0);
  Serial.print(",offset=");
  Serial.print(result.offsetMm, 0);
  Serial.print(",angle=");
  Serial.print(result.angleDeg, 2);
  Serial.print(detectionMode == MODE_BOX ? ",width=" : ",span=");
  Serial.print(result.widthMm, 0);
  Serial.print(",lineError=");
  Serial.print(result.lineErrorMm, 1);
  Serial.print(detectionMode == MODE_BOX ? ",pointsOnBox="
                                        : ",pointsOnWall=");
  Serial.println(result.points);

  if (MONITOR_MODE == 2 && detectionMode == MODE_BOX) {
    printBoxBar(result.offsetMm, result.widthMm);
  }
}

void printMonitorPoints() {
  Serial.println("SCAN_BEGIN");
  for (int i = 0; i < monitorPointCount; i += 3) {
    Serial.print("P,");
    Serial.print(monitorPoints[i].angle, 2);
    Serial.print(",");
    Serial.print(monitorPoints[i].dist, 0);
    Serial.print(",");
    Serial.print(monitorPoints[i].x, 0);
    Serial.print(",");
    Serial.println(monitorPoints[i].y, 0);
  }
  Serial.println("SCAN_END");
}

void updateMonitor() {
  if (MONITOR_MODE == 0 ||
      millis() - lastMonitorTime < MONITOR_INTERVAL_MS) {
    return;
  }

  lastMonitorTime = millis();
  printMonitorSummary();
  if (MONITOR_MODE == 3) {
    printMonitorPoints();
  }
}
