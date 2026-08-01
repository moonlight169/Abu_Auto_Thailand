#include "box_result.h"

BoxResult boxResult = {};
uint32_t lastBoxSeenMs = 0;
uint32_t boxFrameCounter = 0;

uint16_t lidarLastScanPointCount = 0;
uint8_t lostBoxScans = 0;

void clearBoxResult()
{
  boxResult.found = false;
  boxResult.angleDeg = 0.0f;
  boxResult.distanceMm = 9999.0f;
  boxResult.offsetMm = 9999.0f;
  boxResult.widthMm = 0.0f;
  boxResult.lineErrorMm = 9999.0f;
  boxResult.pointCount = 0;
}
