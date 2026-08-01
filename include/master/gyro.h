#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  BNO085 (Adafruit BNO08x) game rotation vector on Wire.
//  Teensy 4.1: SDA pin 18, SCL pin 19. Address 0x4A, falling back to 0x4B.
// ---------------------------------------------------------------------------

extern float gyroHeadingRad;
extern float gyroYawDeg;
extern float gyroRollDeg;
extern float gyroPitchDeg;
extern float gyroRawYawDeg;
extern float gyroYawOffsetDeg;

extern bool gyroOnline;
extern uint32_t lastGyroMs;

bool beginGyro();
void readGyro();
void resetGyroYaw();

void printSensorStatus();
