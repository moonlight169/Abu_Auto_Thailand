#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial7 @ 115200 to the lift controller STM32.
//
//  Master -> lift   LP,<frontPulse>,<backPulse>     move to pulse target
//                   LZ                              home / zero
//  lift -> Master   LIFT_POS,<front>,<back>
//                   LIFT_BUSY | LIFT_REACHED,<f>,<b>
//                   LIFT_HOMING | LIFT_HOME_REACHED
//                   LIFT_ERROR...
//
//  Non-blocking API: startLift() / startLiftHome() + liftDone()/liftFailed().
//  The *AndWait() helpers block and are only safe while the base is parked.
// ---------------------------------------------------------------------------

extern int32_t liftFrontCount;
extern int32_t liftBackCount;
extern int32_t liftTargetFront;
extern int32_t liftTargetBack;
extern int32_t liftPosFront;
extern int32_t liftPosBack;

extern bool liftBusy;
extern bool liftAtTarget;
extern bool liftHomeReached;
extern bool liftMoveActive;
extern bool liftReached;

extern uint32_t liftResponseSequence;

void sendLiftCommandPulse(int32_t front, int32_t back);
void sendLiftCommandZero();
void readLiftSerial();

void startLift(int32_t front, int32_t back,
               uint32_t timeoutMs = LIFT_MOVE_TIMEOUT_MS);
void startLiftHome(uint32_t timeoutMs = LIFT_MOVE_TIMEOUT_MS);

bool liftDone();
bool liftRunning();
bool liftFailed();

// Blocking helpers. Use only while the robot base is stationary.
bool waitLiftAtTarget(uint32_t responseSequenceBeforeCommand,
                      uint32_t timeoutMs);
bool moveLiftPulseAndWait(int32_t front, int32_t back,
                          uint32_t timeoutMs = LIFT_MOVE_TIMEOUT_MS);
bool homeLiftAndWait(uint32_t timeoutMs = LIFT_MOVE_TIMEOUT_MS);
