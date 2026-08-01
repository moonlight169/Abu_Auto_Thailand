#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Vehicle state shared by every module, plus the angle helpers.
// ---------------------------------------------------------------------------

float normalizeAngle(float angle);
float normalizeAngleDeg(float angle);

// Wheel setpoints and the newest feedback from the wheel driver.
extern float targetRPM[WHEEL_COUNT];
extern int32_t encoderCount[WHEEL_COUNT];
extern int32_t previousEncoderCount[WHEEL_COUNT];
extern float actualRPM[WHEEL_COUNT];

// Odometry pose in the world frame.
extern float positionX;
extern float positionY;
extern float encoderHeadingRad;

extern bool commandActive;
extern bool firstFeedback;
extern bool feedbackOnline;
extern bool posePrintEnabled;
extern bool lidarPrintEnabled;
