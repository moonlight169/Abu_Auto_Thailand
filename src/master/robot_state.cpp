#include "robot_state.h"

float targetRPM[WHEEL_COUNT] = {0, 0, 0, 0};
int32_t encoderCount[WHEEL_COUNT] = {0, 0, 0, 0};
int32_t previousEncoderCount[WHEEL_COUNT] = {0, 0, 0, 0};
float actualRPM[WHEEL_COUNT] = {0, 0, 0, 0};

float positionX = 0.0f;
float positionY = 0.0f;
float encoderHeadingRad = 0.0f;

bool commandActive = false;
bool firstFeedback = true;
bool feedbackOnline = false;
bool posePrintEnabled = true;
bool lidarPrintEnabled = true;

float normalizeAngle(float angle)
{
  while (angle > PI)
    angle -= 2.0f * PI;
  while (angle < -PI)
    angle += 2.0f * PI;
  return angle;
}

float normalizeAngleDeg(float angle)
{
  while (angle > 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}
