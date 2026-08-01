#include "lidar.h"

#include <math.h>

DetectionMode detectionMode = MODE_BOX;

static ScanPoint scanPoints[MAX_SCAN_POINTS];
ScanPoint monitorPoints[MAX_SCAN_POINTS];
static int scanCount = 0;
int monitorPointCount = 0;

BoxResult result;
bool filterReady = false;
static float filteredAngle = 0.0f;
static float filteredDistance = 0.0f;
static float filteredOffset = 0.0f;
static float filteredWidth = 0.0f;
int lostCount = 0;

static uint8_t lidarPacket[5];
static uint8_t lidarPacketIndex = 0;
uint32_t validPacketCount = 0;
uint32_t scanFrameCount = 0;
uint32_t lastPacketTime = 0;
uint32_t lastDetectionFrameTime = 0;
int lastClusterCount = 0;
int lastLineCount = 0;

void clearResult() {
  result.found = false;
  result.angleDeg = 0.0f;
  result.distanceMm = 9999.0f;
  result.offsetMm = 9999.0f;
  result.widthMm = 0.0f;
  result.lineErrorMm = 9999.0f;
  result.points = 0;
}

static void sendLidarCommand(uint8_t command) {
  const uint8_t packet[2] = {0xA5, command};
  lidar.write(packet, sizeof(packet));
  lidar.flush();
}

static float makeSignedAngle(float angleDeg) {
  if (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }
  return angleDeg;
}

static float pointDistance(const ScanPoint &a, const ScanPoint &b) {
  const float dx = b.x - a.x;
  const float dy = b.y - a.y;
  return sqrtf(dx * dx + dy * dy);
}

static bool validLidarPacket(const uint8_t *packet) {
  const bool startBit = (packet[0] & 0x01) != 0;
  const bool inverseStartBit = (packet[0] & 0x02) != 0;
  const bool checkBit = (packet[1] & 0x01) != 0;
  return startBit != inverseStartBit && checkBit;
}

static void sortPointsByAngle() {
  for (int i = 1; i < scanCount; i++) {
    ScanPoint temporary = scanPoints[i];
    int j = i - 1;

    while (j >= 0 && scanPoints[j].angle > temporary.angle) {
      scanPoints[j + 1] = scanPoints[j];
      j--;
    }
    scanPoints[j + 1] = temporary;
  }
}

static void saveMonitorSnapshot() {
  monitorPointCount = min(scanCount, MAX_SCAN_POINTS);
  for (int i = 0; i < monitorPointCount; i++) {
    monitorPoints[i] = scanPoints[i];
  }
}

static bool fitLine(int first, int last, BoxResult &output) {
  const int count = last - first + 1;
  const int minimumPoints =
      detectionMode == MODE_BOX ? MIN_BOX_POINTS : MIN_WALL_POINTS;
  if (count < minimumPoints) {
    return false;
  }

  float meanX = 0.0f;
  float meanY = 0.0f;
  for (int i = first; i <= last; i++) {
    meanX += scanPoints[i].x;
    meanY += scanPoints[i].y;
  }
  meanX /= count;
  meanY /= count;

  float sxx = 0.0f;
  float syy = 0.0f;
  float sxy = 0.0f;
  for (int i = first; i <= last; i++) {
    const float dx = scanPoints[i].x - meanX;
    const float dy = scanPoints[i].y - meanY;
    sxx += dx * dx;
    syy += dy * dy;
    sxy += dx * dy;
  }

  if (fabsf(sxx) < 0.001f && fabsf(syy) < 0.001f) {
    return false;
  }

  const float lineAngle = 0.5f * atan2f(2.0f * sxy, sxx - syy);
  const float directionX = cosf(lineAngle);
  const float directionY = sinf(lineAngle);
  float normalX = -directionY;
  float normalY = directionX;

  if (meanX * normalX + meanY * normalY < 0.0f) {
    normalX = -normalX;
    normalY = -normalY;
  }

  float minimumProjection = 999999.0f;
  float maximumProjection = -999999.0f;
  float errorSum = 0.0f;

  for (int i = first; i <= last; i++) {
    const float dx = scanPoints[i].x - meanX;
    const float dy = scanPoints[i].y - meanY;
    const float projection = dx * directionX + dy * directionY;
    minimumProjection = min(minimumProjection, projection);
    maximumProjection = max(maximumProjection, projection);
    errorSum += fabsf(dx * normalX + dy * normalY);
  }

  const float width = maximumProjection - minimumProjection;
  const float averageError = errorSum / count;

  const float maximumLineError =
      detectionMode == MODE_BOX ? MAX_LINE_ERROR_MM
                                : MAX_WALL_LINE_ERROR_MM;

  if (detectionMode == MODE_BOX) {
    if (width < BOX_MIN_WIDTH_MM || width > BOX_MAX_WIDTH_MM) {
      return false;
    }
  }
  if (averageError > maximumLineError) {
    return false;
  }

  float angleDeg = lineAngle * 180.0f / PI;
  while (angleDeg > 90.0f) angleDeg -= 180.0f;
  while (angleDeg < -90.0f) angleDeg += 180.0f;

  const float perpendicularDistance =
      fabsf(meanX * normalX + meanY * normalY);
  const float centerProjection =
      (minimumProjection + maximumProjection) * 0.5f;
  const float centerX = meanX + centerProjection * directionX;

  output.found = true;
  output.angleDeg = angleDeg * ANGLE_SIGN;
  output.distanceMm = perpendicularDistance;
  output.offsetMm = centerX * OFFSET_SIGN;
  output.widthMm = width;
  output.lineErrorMm = averageError;
  output.points = count;
  return true;
}

static void processScan() {
  clearResult();
  scanFrameCount++;
  lastClusterCount = 0;
  lastLineCount = 0;

  const int minimumPoints =
      detectionMode == MODE_BOX ? MIN_BOX_POINTS : MIN_WALL_POINTS;
  if (scanCount < minimumPoints) {
    lostCount++;
    return;
  }

  sortPointsByAngle();
  saveMonitorSnapshot();

  BoxResult bestCandidate;
  bestCandidate.found = false;
  float bestScore = 9999999.0f;
  int clusterStart = 0;

  for (int i = 1; i <= scanCount; i++) {
    bool endCluster = i >= scanCount;

    if (!endCluster) {
      const float pointGap =
          pointDistance(scanPoints[i - 1], scanPoints[i]);
      const float angleGap =
          scanPoints[i].angle - scanPoints[i - 1].angle;
      endCluster =
          pointGap > MAX_CLUSTER_GAP_MM ||
          angleGap > MAX_CLUSTER_ANGLE_GAP_DEG;
    }

    if (!endCluster) {
      continue;
    }

    lastClusterCount++;
    const int clusterEnd = i - 1;
    BoxResult candidate;
    candidate.found = false;

    if (fitLine(clusterStart, clusterEnd, candidate)) {
      lastLineCount++;
      float score;
      if (detectionMode == MODE_BOX) {
        // Box: prefer a clean line near the expected distance and robot center.
        score =
            candidate.lineErrorMm * 5.0f +
            fabsf(candidate.distanceMm - TARGET_DIST_MM) * 0.03f +
            fabsf(candidate.offsetMm) * 0.01f -
            candidate.points * 2.0f;
      } else {
        // Wall/field edge: prefer the longest clean front plane.
        // Small center penalty prevents choosing a long line at the far side.
        score =
            candidate.lineErrorMm * 8.0f +
            fabsf(candidate.offsetMm) * 0.01f -
            candidate.widthMm * 0.08f -
            candidate.points * 2.0f;
      }

      if (!bestCandidate.found || score < bestScore) {
        bestCandidate = candidate;
        bestScore = score;
      }
    }
    clusterStart = i;
  }

  if (!bestCandidate.found) {
    lostCount++;
    if (lostCount >= MAX_LOST_COUNT) {
      filterReady = false;
    }
    return;
  }

  lostCount = 0;
  if (!filterReady) {
    filteredAngle = bestCandidate.angleDeg;
    filteredDistance = bestCandidate.distanceMm;
    filteredOffset = bestCandidate.offsetMm;
    filteredWidth = bestCandidate.widthMm;
    filterReady = true;
  } else {
    filteredAngle +=
        FILTER_ALPHA * (bestCandidate.angleDeg - filteredAngle);
    filteredDistance +=
        FILTER_ALPHA * (bestCandidate.distanceMm - filteredDistance);
    filteredOffset +=
        FILTER_ALPHA * (bestCandidate.offsetMm - filteredOffset);
    filteredWidth +=
        FILTER_ALPHA * (bestCandidate.widthMm - filteredWidth);
  }

  result = bestCandidate;
  result.angleDeg = filteredAngle;
  result.distanceMm = filteredDistance;
  result.offsetMm = filteredOffset;
  result.widthMm = filteredWidth;
  lastDetectionFrameTime = millis();
}

static void processLidarPacket(const uint8_t *packet) {
  const bool startBit = (packet[0] & 0x01) != 0;
  const uint8_t quality = packet[0] >> 2;
  const uint16_t angleQ6 =
      ((uint16_t)packet[1] >> 1) | ((uint16_t)packet[2] << 7);
  const uint16_t distanceQ2 =
      (uint16_t)packet[3] | ((uint16_t)packet[4] << 8);
  const float angleDeg = angleQ6 / 64.0f;
  const float distanceMm = distanceQ2 / 4.0f;

  if (startBit) {
    if (scanCount > 0) {
      processScan();
    }
    scanCount = 0;
  }

  if (quality < MIN_LIDAR_QUALITY) {
    return;
  }

  const float angle = makeSignedAngle(angleDeg);
  if (angle < -VIEW_HALF_ANGLE_DEG || angle > VIEW_HALF_ANGLE_DEG) {
    return;
  }
  if (distanceMm < DETECT_MIN_DIST_MM ||
      distanceMm > DETECT_MAX_DIST_MM ||
      scanCount >= MAX_SCAN_POINTS) {
    return;
  }

  const float angleRad = angle * DEG_TO_RAD;
  ScanPoint &point = scanPoints[scanCount++];
  point.angle = angle;
  point.dist = distanceMm;
  point.x = distanceMm * sinf(angleRad);
  point.y = distanceMm * cosf(angleRad);
}

void readLidar() {
  while (lidar.available() > 0) {
    const uint8_t receivedByte = lidar.read();
    lastPacketTime = millis();
    lidarPacket[lidarPacketIndex++] = receivedByte;

    if (lidarPacketIndex < 5) {
      continue;
    }

    if (validLidarPacket(lidarPacket)) {
      validPacketCount++;
      processLidarPacket(lidarPacket);
      lidarPacketIndex = 0;
    } else {
      lidarPacket[0] = lidarPacket[1];
      lidarPacket[1] = lidarPacket[2];
      lidarPacket[2] = lidarPacket[3];
      lidarPacket[3] = lidarPacket[4];
      lidarPacketIndex = 4;
    }
  }
}

void resetDetectionFilter() {
  filterReady = false;
  lostCount = 0;
  lastDetectionFrameTime = 0;
  clearResult();
}

void setDetectionMode(DetectionMode newMode) {
  detectionMode = newMode;
  resetDetectionFilter();
  Serial.print("LIDAR MODE=");
  Serial.println(detectionMode == MODE_BOX ? "BOX" : "WALL");
}

void beginLidar() {
  lidar.begin(LIDAR_BAUD);
  delay(1000);

  // Same LiDAR startup sequence as the supplied working example.
  sendLidarCommand(0x40);
  delay(2000);
  sendLidarCommand(0x25);
  delay(500);
  sendLidarCommand(0x20);
  delay(500);
}
