#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Low-speed parking against a box face (or a wall) using the detection the
//  Hub reports inside the "HUB,..." packet.
//
//  Three axes are closed at once: distance -> vx, lateral offset -> vy,
//  face angle -> wz. The step finishes when all three stay inside tolerance
//  for PARK_DONE_HOLD_MS. In wall mode the offset axis is ignored, because a
//  long wall has no meaningful centre.
// ---------------------------------------------------------------------------

extern bool boxParkingActive;
extern bool wallAlignmentMode;
extern bool hubDetectionIsWall;
extern float boxParkingTargetMm;

extern float parkingVx;
extern float parkingVy;
extern float parkingWz;

// Cleared by the mission RESET steps as well as by startBoxParking().
extern uint32_t boxParkingInToleranceSinceMs;

void startBoxParking(float targetDistanceMm, uint32_t timeoutMs,
                     bool alignWall = false);
void startWallAlignment(float targetDistanceMm, uint32_t timeoutMs);

bool boxParkingDone();
bool boxParkingFailed();

void updateBoxParking();
void updateLidarPrint();
