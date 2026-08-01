#include "master_link.h"

#include "io_board.h"
#include "lidar.h"
#include "tfmini.h"

// Send one complete sensor-hub packet to Master continuously at 20 Hz.
bool masterStreamEnabled = true;
uint32_t masterStreamIntervalMs = 50;

static uint32_t lastMasterStreamTime = 0;

void sendMasterLidar() {
  const bool fresh =
      result.found &&
      lastDetectionFrameTime != 0 &&
      millis() - lastDetectionFrameTime <= 300;

  master.print("LIDAR,");
  master.print(detectionMode == MODE_BOX ? "BOX," : "WALL,");
  if (!fresh) {
    master.println("0");
    return;
  }

  master.print("1,");
  master.print(result.distanceMm, 0);
  master.print(',');
  master.print(result.offsetMm, 0);
  master.print(',');
  master.print(result.angleDeg, 2);
  master.print(',');
  master.print(result.widthMm, 0);
  master.print(',');
  master.println(result.points);
}

void sendMasterIo() {
  master.print("IO,");
  master.println(getInputActiveMask());
}

void sendMasterServo() {
  master.print("SERVO,");
  master.print(armAngleDeg);
  master.print(',');
  master.println(spinAngleDeg);
}

void sendMasterTfmini() {
  master.print("TFMINI,");
  for (uint8_t i = 0; i < TFMINI_COUNT; i++) {
    master.print(tfminiFresh(i) ? tfminiData[i].distanceCm : 0);
    master.print(',');
  }
  master.println(getTfminiValidMask());
}

void sendMasterStatus() {
  const bool fresh =
      result.found &&
      lastDetectionFrameTime != 0 &&
      millis() - lastDetectionFrameTime <= 300;

  master.print("HUB,");
  master.print(millis());
  master.print(',');
  master.print(detectionMode == MODE_BOX ? "BOX" : "WALL");
  master.print(',');
  master.print(fresh ? 1 : 0);
  master.print(',');

  if (fresh) {
    master.print(result.distanceMm, 0);
    master.print(',');
    master.print(result.offsetMm, 0);
    master.print(',');
    master.print(result.angleDeg, 2);
    master.print(',');
    master.print(result.widthMm, 0);
    master.print(',');
    master.print(result.points);
  } else {
    // Empty LiDAR fields prevent Master from reusing an old detection.
    master.print("0,0,0.00,0,0");
  }

  master.print(',');
  master.print(getInputActiveMask());
  master.print(',');
  master.print(getRelayOnMask());
  master.print(',');
  master.print(armAngleDeg);
  master.print(',');
  master.print(spinAngleDeg);
  master.print(',');

  // TFMini-S 1/2/3/4 distance in cm. Zero means no fresh valid frame.
  for (uint8_t i = 0; i < TFMINI_COUNT; i++) {
    master.print(tfminiFresh(i) ? tfminiData[i].distanceCm : 0);
    master.print(',');
  }
  master.println(getTfminiValidMask());
}

void updateMasterStream() {
  if (!masterStreamEnabled ||
      millis() - lastMasterStreamTime < masterStreamIntervalMs) {
    return;
  }
  lastMasterStreamTime = millis();
  sendMasterStatus();
}
