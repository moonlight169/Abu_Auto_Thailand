#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  World-frame PD position controller: drive to (X, Y, Yaw) using odometry.
//  Used by the "P,x,y,yaw" console command and by MOVE_STEP() in a mission.
// ---------------------------------------------------------------------------

extern bool positionControlActive;

extern float targetPositionX;
extern float targetPositionY;
extern float targetPositionYawRad;

extern float previousErrorX;
extern float previousErrorY;
extern float previousErrorYaw;

extern uint8_t positionDoneCounter;
extern float currentMoveMaxSpeed;

void setPositionTarget(float x, float y, float yawDeg);
void startMoveTo(float x, float y, float yawDeg,
                 uint32_t timeoutMs = 15000,
                 float maxSpeed = MAX_POSITION_SPEED);

bool moveDone();
bool moveRunning();
bool moveFailed();

void cancelPositionControl();
void updatePositionControl();
