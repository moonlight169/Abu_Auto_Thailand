#pragma once

#include <Arduino.h>

// ===========================================================================
//  Master - shared enums and structs.
//  No tuning numbers live here; those are in config.h.
// ===========================================================================

#define ARRAY_COUNT(arrayName) (sizeof(arrayName) / sizeof((arrayName)[0]))

enum WheelIndex : uint8_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
  WHEEL_COUNT = 4
};

enum VelocityMode : uint8_t
{
  VELOCITY_STOPPED,
  VELOCITY_LOCAL,
  VELOCITY_GLOBAL
};

enum TaskStatus : uint8_t
{
  TASK_IDLE,
  TASK_RUNNING,
  TASK_DONE,
  TASK_TIMEOUT,
  TASK_ERROR
};

// --------------------------------------------------------------- LiDAR ----
// Filled from the HUB packet. See box_result.h.
struct BoxResult
{
  bool found;
  float distanceMm;
  float offsetMm;
  float angleDeg;
  float widthMm;
  float lineErrorMm;
  uint16_t pointCount;
};

// ----------------------------------------------------------------- Hub ----
struct HubState
{
  bool online;
  uint32_t slaveTimeMs;
  uint16_t inputMask;
  uint8_t relayMask;
  uint8_t armAngle;
  uint8_t spinAngle;
  uint16_t tfDistanceCm[4];
  uint8_t tfValidMask;
  uint32_t lastPacketMs;
  uint32_t goodPackets;
  uint32_t badPackets;
};

// Bit positions inside HubState::inputMask.
enum HubInputBit : uint8_t
{
  HUB_L_SW_FRONT = 0,
  HUB_LDR2 = 1,  // inputMask value 2
  HUB_LDR1 = 2,  // inputMask value 4 (verified on the actual HUB)
  HUB_LASER5 = 3,
  HUB_SW_GREEN = 4,
  HUB_SW_BLUE = 5,
  HUB_SW_RED = 6,
  HUB_SW_A = HUB_SW_RED,  // Backward-compatible alias
  HUB_SW_YELLOW = 7,
  HUB_R_SW_FRONT = 8
};

// ------------------------------------------------ higher-step alignment ---
enum HigherAlignState : uint8_t
{
  HIGH_ALIGN_IDLE,
  HIGH_ALIGN_START_LIFT,
  HIGH_ALIGN_APPROACH,
  HIGH_ALIGN_CONFIRM_BOTH,
  HIGH_ALIGN_BACKOFF,
  HIGH_ALIGN_AFTER_LIFT,
  HIGH_ALIGN_LIDAR,
  HIGH_ALIGN_NO_BOX_CONTACT_DRIVE,
  HIGH_ALIGN_NO_BOX_FRONT_ZERO,
  HIGH_ALIGN_NO_BOX_FORWARD_TF2,
  HIGH_ALIGN_NO_BOX_BACK_ZERO,
  HIGH_ALIGN_NO_BOX_RETURN_START_HEIGHT,
  HIGH_ALIGN_NO_BOX_FORWARD_TF1
};

// --------------------------------------------------------- external arm ---
enum ArmTaskStatus : uint8_t
{
  ARM_TASK_IDLE,
  ARM_TASK_RUNNING,
  ARM_TASK_DONE,
  ARM_TASK_ERROR
};

enum ArmResetState : uint8_t
{
  ARM_RESET_IDLE,
  ARM_RESET_WAIT_CLEAR,
  ARM_RESET_WAIT_HOME
};

// --------------------------------------------------- mission sub-states ---
enum ForwardLzBackState : uint8_t
{
  FORWARD_LZ_BACK_IDLE,
  FORWARD_LZ_BACK_FORWARD,
  FORWARD_LZ_BACK_WAIT_HOME,
  FORWARD_LZ_BACK_REVERSE,
  FORWARD_LZ_BACK_DONE
};

enum Laser5PickState : uint8_t
{
  LASER5_PICK_IDLE,
  LASER5_PICK_SEARCHING,
  LASER5_PICK_WAIT_VERIFY,
  LASER5_PICK_WAIT_GRIPPER_OPEN,
  LASER5_PICK_WAIT_ARM_RETURN,
  LASER5_PICK_DONE
};
