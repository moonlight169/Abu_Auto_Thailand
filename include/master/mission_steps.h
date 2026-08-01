#pragma once

#include "config.h"

// ===========================================================================
//  Mission step vocabulary.
//
//  A mission is a plain array of MissionStep. The runner starts one row,
//  waits until it reports done, then moves to the next. Build the rows with
//  the macros below and put the array in mission_list.cpp.
//
//  To add a NEW kind of step:
//    1. add a value to MissionStepType
//    2. add a *_STEP() macro here
//    3. handle it in startCurrentMissionStep() / currentMissionStepDone() /
//       currentMissionStepFailed() in mission_runner.cpp
// ===========================================================================

enum MissionStepType : uint8_t
{
  STEP_RESET,
  STEP_RESET_2,
  STEP_LIFT,
  STEP_MOVE,
  STEP_LIDAR_PARK,
  STEP_WALL_ALIGN,
  STEP_MOVE_UNTIL_FRONT,
  STEP_FORWARD_UNTIL_LASER3,
  STEP_FORWARD_UNTIL_LASER4,
  STEP_FORWARD_TF34_EDGE_FRONT_LIFT,
  STEP_FORWARD_TF1_BACK_LIFT,
  STEP_FORWARD_LZ_BACK,
  STEP_SLIDE_UNTIL_LASER5,
  STEP_LASER5_PICK_VERIFY,
  STEP_WAIT,
  STEP_HUB_WAIT_INPUT,
  STEP_HUB_RELAY,
  STEP_HUB_ARM,
  STEP_HUB_SPIN,
  STEP_ARM_HOME,
  STEP_ARM_BOTTOM,
  STEP_ARM_TOP,
  STEP_ARM_POS,
  STEP_END
};

struct MissionStep
{
  MissionStepType type;
  float p1;
  float p2;
  float p3;
  float maxSpeed;
  uint32_t timeoutMs;
};

struct MissionProgram
{
  const char *name;
  const MissionStep *steps;
  size_t stepCount;
};

// ---------------------------------------------------------------------------
//  Step builders
// ---------------------------------------------------------------------------

#define RESET_STEP() \
  { STEP_RESET, 0.0f, 0.0f, 0.0f, 0.0f, 0 }

#define RESET_STEP_2() \
  { STEP_RESET_2, 0.0f, 0.0f, 0.0f, 0.0f, 0 }

#define LIFT_STEP(front, back) \
  { STEP_LIFT, (float)(front), (float)(back), 0.0f, 0.0f, LIFT_MOVE_TIMEOUT_MS }

#define MOVE_STEP(x, y, yawDeg, maxSpeed) \
  { STEP_MOVE, (float)(x), (float)(y), (float)(yawDeg), \
    (float)(maxSpeed), DEFAULT_MOVE_TIMEOUT_MS }

#define LIDAR_PARK_STEP(distanceMm, timeoutMs) \
  { STEP_LIDAR_PARK, (float)(distanceMm), 0.0f, 0.0f, \
    0.0f, (uint32_t)(timeoutMs) }

#define WALL_ALIGN_STEP(distanceMm, timeoutMs) \
  { STEP_WALL_ALIGN, (float)(distanceMm), 0.0f, 0.0f, \
    0.0f, (uint32_t)(timeoutMs) }

#define FRONT_LIMIT_STEP(forwardSpeed, timeoutMs) \
  { STEP_MOVE_UNTIL_FRONT, (float)(forwardSpeed), 0.0f, 0.0f, \
    0.0f, (uint32_t)(timeoutMs) }

#define LASER3_FORWARD_STEP(forwardSpeed, timeoutMs) \
  { STEP_FORWARD_UNTIL_LASER3, (float)(forwardSpeed), 0.0f, 0.0f, \
    0.0f, (uint32_t)(timeoutMs) }

#define LASER4_FORWARD_STEP(forwardSpeed, timeoutMs) \
  { STEP_FORWARD_UNTIL_LASER4, (float)(forwardSpeed), 0.0f, 0.0f, \
    0.0f, (uint32_t)(timeoutMs) }

// Drive forward until TF3 and TF4 change from seeing floor to no floor.
// Stop first, then command LP,<frontPulse>,<backPulse> and wait for the lift.
#define TF34_EDGE_FRONT_LIFT_STEP(forwardSpeed, frontPulse, backPulse, timeoutMs) \
  { STEP_FORWARD_TF34_EDGE_FRONT_LIFT, (float)(forwardSpeed), \
    (float)(frontPulse), (float)(backPulse), 0.0f, (uint32_t)(timeoutMs) }

// Drive until the single TF1 sensor detects a surface. Stop all wheels,
// keep the current front-lift position and move only the back lift.
#define TF1_FORWARD_BACK_LIFT_STEP(forwardSpeed, backPulse, timeoutMs) \
  { STEP_FORWARD_TF1_BACK_LIFT, (float)(forwardSpeed), \
    (float)(backPulse), 0.0f, 0.0f, (uint32_t)(timeoutMs) }

// Drive forward by time, home the lift with LZ and wait for
// LIFT_HOME_REACHED, then drive backward by time and stop.
#define FORWARD_LZ_BACK_STEP(forwardSpeed, forwardMs, backSpeed, backMs, timeoutMs) \
  { STEP_FORWARD_LZ_BACK, (float)(forwardSpeed), (float)(forwardMs), \
    (float)(backSpeed), (float)(backMs), (uint32_t)(timeoutMs) }

#define LASER5_SLIDE_STEP(slideSpeed, forwardSpeed, timeoutMs) \
  { STEP_SLIDE_UNTIL_LASER5, (float)(slideSpeed), \
    (float)(forwardSpeed), 0.0f, 0.0f, (uint32_t)(timeoutMs) }

// Slide until Laser5 sees the workpiece, close Relay4, move Arm to 0,
// then verify Laser5 again. If verification fails, open the gripper, move the
// Arm back to armReturnAngle and resume sliding until the timeout expires.
//
// armReturnAngle is the pick-ready arm position in degrees, 0..HUB_ARM_MAX_DEG.
// It is a per-mission value so the same step works on both fields.
#define LASER5_PICK_VERIFY_STEP(slideSpeed, forwardSpeed, armReturnAngle, timeoutMs) \
  { STEP_LASER5_PICK_VERIFY, (float)(slideSpeed), \
    (float)(forwardSpeed), (float)(armReturnAngle), 0.0f, (uint32_t)(timeoutMs) }

#define WAIT_STEP(waitMs) \
  { STEP_WAIT, 0.0f, 0.0f, 0.0f, 0.0f, (uint32_t)(waitMs) }

#define HUB_WAIT_INPUT_STEP(bit, active, timeoutMs) \
  { STEP_HUB_WAIT_INPUT, (float)(bit), (float)(active), 0.0f, 0.0f, (uint32_t)(timeoutMs) }

// Stop at this mission step until LDR1 becomes ON.
// timeoutMs = 0 means wait forever.
#define LDR1_WAIT_STEP(timeoutMs) \
  HUB_WAIT_INPUT_STEP(HUB_LDR1, 1, timeoutMs)

#define HUB_RELAY_STEP(relay, on) \
  { STEP_HUB_RELAY, (float)(relay), (float)(on), 0.0f, 0.0f, 0 }

#define HUB_ARM_STEP(angle) \
  { STEP_HUB_ARM, (float)(angle), 0.0f, 0.0f, 0.0f, 0 }

#define HUB_SPIN_STEP(angle) \
  { STEP_HUB_SPIN, (float)(angle), 0.0f, 0.0f, 0.0f, 0 }

#define ARM_HOME_STEP() \
  { STEP_ARM_HOME, 0.0f, 0.0f, 0.0f, 0.0f, ARM_DEFAULT_TIMEOUT_MS }

#define ARM_BOTTOM_STEP(angle) \
  { STEP_ARM_BOTTOM, (float)(angle), 0.0f, 0.0f, 0.0f, ARM_DEFAULT_TIMEOUT_MS }

#define ARM_TOP_STEP(angle) \
  { STEP_ARM_TOP, (float)(angle), 0.0f, 0.0f, 0.0f, ARM_DEFAULT_TIMEOUT_MS }

#define ARM_POS_STEP(bottomAngle, topAngle) \
  { STEP_ARM_POS, (float)(bottomAngle), (float)(topAngle), \
    0.0f, 0.0f, ARM_DEFAULT_TIMEOUT_MS }

#define END_STEP() \
  { STEP_END, 0.0f, 0.0f, 0.0f, 0.0f, 0 }
