#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Box / wall detection result.
//
//  The RPLiDAR is connected to the HUB, not to the Master. The Hub does all
//  the scan processing and sends the finished detection inside the "HUB,..."
//  packet; hub_link.cpp fills the values below, and box_parking.cpp /
//  high_align.cpp drive the robot from them.
//
//  Tune the detection itself in the HUB firmware (src/slave_hub/config.cpp),
//  not here.
//
//  Note: the Master used to parse raw LiDAR packets itself on Serial6. That
//  code was already dead (nothing called it) and Serial6 is the Arm link now,
//  so it was removed. The original is still in Code/ABU_Auto_Master.ino.
// ---------------------------------------------------------------------------

extern BoxResult boxResult;
extern uint32_t lastBoxSeenMs;
extern uint32_t boxFrameCounter;

// Shown by updateLidarPrint().
extern uint16_t lidarLastScanPointCount;
extern uint8_t lostBoxScans;

void clearBoxResult();
