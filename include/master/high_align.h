#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  First stair routine: align the robot against a higher step and zero yaw.
//  It intentionally uses no X/Y position or encoder travel target.
//
//  START LIFT -> APPROACH (crawl one side at a time until both front limits)
//  -> CONFIRM -> gyro zero -> BACKOFF -> soft AFTER LIFT -> LiDAR box park.
//  If no box is found it falls back to the TF2/TF1 timed sequence.
//
//  Console: "HIGH START" / "HIGH START,<front>,<back>" / "HIGH STOP".
// ---------------------------------------------------------------------------

extern bool higherAlignActive;

bool startHigherStepAlignment(int32_t afterLiftFrontPulse,
                              int32_t afterLiftBackPulse);
void stopHigherStepAlignment(const char *reason);
void updateHigherStepAlignment();
