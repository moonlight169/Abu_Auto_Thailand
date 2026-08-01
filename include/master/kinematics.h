#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Mecanum inverse kinematics, the V/G velocity ramp, and the global stop.
//
//  setTargets()          raw per-wheel RPM
//  setLocalVelocity()    body frame vx/vy/wz  -> wheel RPM
//  setGlobalVelocity()   world frame vx/vy/wz -> rotated into the body frame
//  commandLocal/Global() latch a target that updateVelocityControl() ramps to
// ---------------------------------------------------------------------------

extern VelocityMode velocityMode;

extern float commandedVx;
extern float commandedVy;
extern float commandedWz;

extern float rampedVx;
extern float rampedVy;
extern float rampedWz;

void setTargets(float fl, float fr, float rl, float rr);
void setLocalVelocity(float vxLocal, float vyLocal, float wz);
void setGlobalVelocity(float vxGlobal, float vyGlobal, float wz);

void commandLocalVelocity(float vx, float vy, float wz);
void commandGlobalVelocity(float vx, float vy, float wz);

float rampValue(float current, float target,
                float accelRate, float decelRate, float dt);
bool rampIsStopped();
void updateSoftVelocity(float targetVx, float targetVy,
                        float targetWz, bool globalMode);
void updateVelocityControl();

void stopRobot();

float getRobotYawRad();
void limitGlobalVelocity(float &vx, float &vy, float maxSpeed);
