#pragma once

#include "mission_steps.h"

// Registry of every mission program. The tables themselves are in
// mission_list.cpp -- that is the file to edit when changing a run.

extern const MissionProgram missionPrograms[];
extern const size_t MISSION_PROGRAM_COUNT;

// 0 = Mission 1. Changed with "SEL n" or "Mn" on the console.
extern uint8_t selectedMission;

const MissionProgram &activeMissionProgram();

// Exact-match lookup on the `name` field. Returns the index into
// missionPrograms[], or -1 when nothing matches. This is what lets the keypad
// codes and the Hub buttons address a mission by name instead of by position,
// so adding or reordering entries can never point a button somewhere else.
int findMissionByName(const char *name);

// Reports every duplicated name to the USB console and returns true when they
// are all unique. Two entries with the same name compile without a warning and
// the first one silently wins every lookup, so main.cpp runs this once at boot.
bool checkMissionNamesUnique();

//-------------------------------- config mission list ---------------------------------

// GLOBAL
constexpr int32_t LIFT_START_FRONT = 100; 
constexpr int32_t LIFT_START_BACK = 200;

constexpr int32_t LIFT_DOWN_FRONT = 100; 
constexpr int32_t LIFT_DOWN_BACK = 200;

constexpr uint32_t WAIT_LIFT_LEVEL_MS = 500;

constexpr int32_t LIFT_LV1_FRONT = 1000;
constexpr int32_t LIFT_LV1_BACK = 1100;

constexpr int32_t LIFT_LV2_FRONT = 2000;
constexpr int32_t LIFT_LV2_BACK = 2100;

constexpr int32_t LIFT_LV3_FRONT = 3800;
constexpr int32_t LIFT_LV3_BACK = 3900;

constexpr int32_t LIFT_ZERO = 0;

//-----------------------------------------

constexpr int32_t TIMEOUT_LIDAR = 30000;
constexpr int32_t DISTANCE_LIDAR = 385;

constexpr float MOVE_FORWARD_BEFORELIDAR_BLUE = 0.4;
constexpr float MOVE_SLIDE_BEFORELIDAR_BLUE = 0.0;
constexpr float MOVE_YAW_BEFORELIDAR_BLUE = 0.0;
constexpr float MOVE_SPEED_BEFORELIDAR_BLUE = 0.5;

constexpr float MOVE_FORWARD_BEFORELIDAR_RED = 0.4;
constexpr float MOVE_SLIDE_BEFORELIDAR_RED = 0.0;
constexpr float MOVE_YAW_BEFORELIDAR_RED = 0.0;
constexpr float MOVE_SPEED_BEFORELIDAR_RED = 0.5;

//-----------------------------------------

constexpr int32_t RELAY_ARM = 1;
constexpr int32_t RELAY1_ON = 1;
constexpr int32_t RELAY1_OFF = 0;

constexpr int32_t RELAY_FRONT = 2;
constexpr int32_t RELAY2_ON = 0;
constexpr int32_t RELAY2_OFF = 1;

constexpr int32_t RELAY_BACK = 3;
constexpr int32_t RELAY3_ON = 0;
constexpr int32_t RELAY3_OFF = 1;

constexpr int32_t RELAY4_ON = 1;
constexpr int32_t RELAY4_OFF = 0;

constexpr uint32_t WAIT_STEP_MS = 100;

//-----------------------------------------

constexpr int32_t ARM_MIDDLE_DEG = 100;

constexpr int32_t ARM_FRONT_UP_DEG = 25;
constexpr int32_t ARM_FRONT_DOWN_DEG = 30;

constexpr int32_t ARM_BACK_UP_DEG = 175;
constexpr int32_t ARM_BACK_DOWN_DEG = 180;

constexpr int32_t BOTTOM_MIDDLE_DEG = 90;

constexpr int32_t BOTTOM_FRONT_DEG = 5;
constexpr int32_t BOTTOM_BACK_DEG = 180;

//-----------------------------------------

constexpr float FORWARD_SPEED_LASER3 = 0.5;
constexpr uint32_t TIMEOUT_LASER3 = 15000;

constexpr float FORWARD_SPEED_LASER4 = 0.5;
constexpr uint32_t TIMEOUT_LASER4 = 15000;

constexpr float FORWARD_SPEED_TOF34 = 0.30f;
constexpr int32_t FRONT_PULSE_TOF34 = 1800;
constexpr int32_t BACK_PULSE_TOF34 = 0;
constexpr uint32_t TIMEOUT_TOF34 = 15000;

constexpr float FORWARD_SPEED_FLOOR_TOF34 = 0.30f;
constexpr int32_t FRONT_PULSE_FLOOR_TOF34 = 3800;
constexpr int32_t BACK_PULSE_FLOOR_TOF34 = 0;
constexpr uint32_t TIMEOUT_FLOOR_TOF34 = 15000;

constexpr float FORWARD_SPEED_TF1 = 0.30f;
constexpr int32_t BACK_PULSE_TF1 = 1800;
constexpr uint32_t TIMEOUT_TF1 = 15000;

constexpr float FORWARD_SPEED_FLOOR_TF1 = 0.30f;
constexpr int32_t BACK_PULSE_FLOOR_TF1 = 3800;
constexpr uint32_t TIMEOUT_FLOOR_TF1 = 15000;

//-----------------------------------------

constexpr float FORWARD_SPEED_LZ_BACK = 0.50f;
constexpr uint32_t FORWARDTIME_LZ_BACK = 1500;
constexpr float BACK_SPEED_LZ_BACK = 0.30f;
constexpr uint32_t BACKTIME_LZ_BACK = 2000;
constexpr uint32_t TIMEOUT_LZ_BACK = 15000;

//-----------------------------------------

constexpr float FORWARD_SPEED_LIMIT = 0.5;
constexpr uint32_t TIMEOUT_LIMIT = 15000;

constexpr float MOVE_FORWARD_AFTERLIMIT_BLUE = 0.3;
constexpr float MOVE_SLIDE_AFTERLIMIT_BLUE = 0.1;
constexpr float MOVE_YAW_AFTERLIMIT_BLUE = 0.0;
constexpr float MOVE_SPEED_AFTERLIMIT_BLUE = 0.5;

constexpr float MOVE_FORWARD_AFTERLIMIT_RED = 0.3;
constexpr float MOVE_SLIDE_AFTERLIMIT_RED = -0.1;
constexpr float MOVE_YAW_AFTERLIMIT_RED = 0.0;
constexpr float MOVE_SPEED_AFTERLIMIT_RED = 0.5;

constexpr float MOVE_TIME_SPEED = 0.3;
constexpr uint32_t MOVE_TIMER_MS = 3000;

// FIELD BLUE LINE 1
constexpr float MOVE_START_FORWARD_L1_BLUE = 3.8;
constexpr float MOVE_START_SLIDE_L1_BLUE = 5.8;
constexpr float MOVE_START_YAW_L1_BLUE = 0.0;
constexpr float MOVE_START_SPEED_L1_BLUE = 1.0;

// constexpr float MOVE_START_FORWARD_L1_NOLIDAR_BLUE = 4.9;
// constexpr float MOVE_START_SLIDE_L1_NOLIDAR_BLUE = 5.8;
// constexpr float MOVE_START_YAW_L1_NOLIDAR_BLUE = 0.0;
// constexpr float MOVE_START_SPEED_L1_NOLIDAR_BLUE = 1.0;

constexpr float MOVE_SETUP_FORWARD_L1_BLUE = -1.0;
constexpr float MOVE_SETUP_SLIDE_L1_BLUE = 0.0;
constexpr float MOVE_SETUP_YAW_L1_BLUE = 0.0;
constexpr float MOVE_SETUP_SPEED_L1_BLUE = 1.0;

constexpr float MOVE_STANDBY_FORWARD_L1_BLUE = -1.0;
constexpr float MOVE_STANDBY_SLIDE_L1_BLUE = 1.69;
constexpr float MOVE_STANDBY_YAW_L1_BLUE = 0.0;
constexpr float MOVE_STANDBY_SPEED_L1_BLUE = 1.0;

constexpr float MOVE_RAMP_FORWARD_L1_BLUE = 10.0;
constexpr float MOVE_RAMP_SLIDE_L1_BLUE = 1.69;
constexpr float MOVE_RAMP_YAW_L1_BLUE = 0.0;
constexpr float MOVE_RAMP_SPEED_L1_BLUE = 1.5;

// FIELD BLUE LINE 2
constexpr float MOVE_START_FORWARD_L2_BLUE = 3.8;
constexpr float MOVE_START_SLIDE_L2_BLUE = 2.9;
constexpr float MOVE_START_YAW_L2_BLUE = 0.0;
constexpr float MOVE_START_SPEED_L2_BLUE = 1.5;

// constexpr float MOVE_START_FORWARD_L2_NOLIDAR_BLUE = 4.9;
// constexpr float MOVE_START_SLIDE_L2_NOLIDAR_BLUE = 2.9;
// constexpr float MOVE_START_YAW_L2_NOLIDAR_BLUE = 0.0;
// constexpr float MOVE_START_SPEED_L2_NOLIDAR_BLUE = 1.5;

constexpr float MOVE_SETUP_FORWARD_L2_BLUE = -1.0;
constexpr float MOVE_SETUP_SLIDE_L2_BLUE = 0.0;
constexpr float MOVE_SETUP_YAW_L2_BLUE = 0.0;
constexpr float MOVE_SETUP_SPEED_L2_BLUE = 1.5;

constexpr float MOVE_STANDBY_FORWARD_L2_BLUE = -1.0;
constexpr float MOVE_STANDBY_SLIDE_L2_BLUE = 4.59;
constexpr float MOVE_STANDBY_YAW_L2_BLUE = 0.0;
constexpr float MOVE_STANDBY_SPEED_L2_BLUE = 1.5;

constexpr float MOVE_RAMP_FORWARD_L2_BLUE = 10.0;
constexpr float MOVE_RAMP_SLIDE_L2_BLUE = 4.59;
constexpr float MOVE_RAMP_YAW_L2_BLUE = 0.0;
constexpr float MOVE_RAMP_SPEED_L2_BLUE = 1.5;


// FIELD BLUE LINE 3
constexpr float MOVE_START_FORWARD_L3_BLUE = 3.8;
constexpr float MOVE_START_SLIDE_L3_BLUE = 0.5;
constexpr float MOVE_START_YAW_L3_BLUE = 0.0;
constexpr float MOVE_START_SPEED_L3_BLUE = 1.5;

// constexpr float MOVE_START_FORWARD_L3_NOLIDAR_BLUE = 4.9;
// constexpr float MOVE_START_SLIDE_L3_NOLIDAR_BLUE = 0.6;
// constexpr float MOVE_START_YAW_L3_NOLIDAR_BLUE = 0.0;
// constexpr float MOVE_START_SPEED_L3_NOLIDAR_BLUE = 1.5;

constexpr float MOVE_SETUP_FORWARD_L3_BLUE = -1.0;
constexpr float MOVE_SETUP_SLIDE_L3_BLUE = 0.0;
constexpr float MOVE_SETUP_YAW_L3_BLUE = 0.0;
constexpr float MOVE_SETUP_SPEED_L3_BLUE = 1.5;

constexpr float MOVE_STANDBY_FORWARD_L3_BLUE = -1.0;
constexpr float MOVE_STANDBY_SLIDE_L3_BLUE = 7.49;
constexpr float MOVE_STANDBY_YAW_L3_BLUE = 0.0;
constexpr float MOVE_STANDBY_SPEED_L3_BLUE = 1.5;

constexpr float MOVE_RAMP_FORWARD_L3_BLUE = 10.0;
constexpr float MOVE_RAMP_SLIDE_L3_BLUE = 7.49;
constexpr float MOVE_RAMP_YAW_L3_BLUE = 0.0;
constexpr float MOVE_RAMP_SPEED_L3_BLUE = 1.5;

// FIELD RED LINE 1
constexpr float MOVE_START_FORWARD_L1_RED = 3.8;
constexpr float MOVE_START_SLIDE_L1_RED = -5.8;
constexpr float MOVE_START_YAW_L1_RED = 0.0;
constexpr float MOVE_START_SPEED_L1_RED = 1.5;

// constexpr float MOVE_START_FORWARD_L1_NOLIDAR_RED = 4.9;
// constexpr float MOVE_START_SLIDE_L1_NOLIDAR_RED = -5.8;
// constexpr float MOVE_START_YAW_L1_NOLIDAR_RED = 0.0;
// constexpr float MOVE_START_SPEED_L1_NOLIDAR_RED = 1.5;

constexpr float MOVE_SETUP_FORWARD_L1_RED = -1.0;
constexpr float MOVE_SETUP_SLIDE_L1_RED = 0.0;
constexpr float MOVE_SETUP_YAW_L1_RED = 0.0;
constexpr float MOVE_SETUP_SPEED_L1_RED = 1.5;

constexpr float MOVE_STANDBY_FORWARD_L1_RED = -1.0;
constexpr float MOVE_STANDBY_SLIDE_L1_RED = -1.69;
constexpr float MOVE_STANDBY_YAW_L1_RED = 0.0;
constexpr float MOVE_STANDBY_SPEED_L1_RED = 1.5;

constexpr float MOVE_RAMP_FORWARD_L1_RED = 10.0;
constexpr float MOVE_RAMP_SLIDE_L1_RED = -1.69;
constexpr float MOVE_RAMP_YAW_L1_RED = 0.0;
constexpr float MOVE_RAMP_SPEED_L1_RED = 1.5;

// FIELD RED LINE 2
constexpr float MOVE_START_FORWARD_L2_RED = 3.8;
constexpr float MOVE_START_SLIDE_L2_RED = -3.3;
constexpr float MOVE_START_YAW_L2_RED = 0.0;
constexpr float MOVE_START_SPEED_L2_RED = 1.5;

// constexpr float MOVE_START_FORWARD_L2_NOLIDAR_RED = 4.9;
// constexpr float MOVE_START_SLIDE_L2_NOLIDAR_RED = -3.3;
// constexpr float MOVE_START_YAW_L2_NOLIDAR_RED = 0.0;
// constexpr float MOVE_START_SPEED_L2_NOLIDAR_RED = 1.5;

constexpr float MOVE_SETUP_FORWARD_L2_RED = -1.0;
constexpr float MOVE_SETUP_SLIDE_L2_RED = 0.0;
constexpr float MOVE_SETUP_YAW_L2_RED = 0.0;
constexpr float MOVE_SETUP_SPEED_L2_RED = 1.5;

constexpr float MOVE_STANDBY_FORWARD_L2_RED = -1.0;
constexpr float MOVE_STANDBY_SLIDE_L2_RED = -4.59;
constexpr float MOVE_STANDBY_YAW_L2_RED = 0.0;
constexpr float MOVE_STANDBY_SPEED_L2_RED = 1.5;

constexpr float MOVE_RAMP_FORWARD_L2_RED = 10.0;
constexpr float MOVE_RAMP_SLIDE_L2_RED = -4.59;
constexpr float MOVE_RAMP_YAW_L2_RED = 0.0;
constexpr float MOVE_RAMP_SPEED_L2_RED = 1.5;


// FIELD RED LINE 3
constexpr float MOVE_START_FORWARD_L3_RED = 3.8;
constexpr float MOVE_START_SLIDE_L3_RED = -0.5;
constexpr float MOVE_START_YAW_L3_RED = 0.0;
constexpr float MOVE_START_SPEED_L3_RED = 1.5;

// constexpr float MOVE_START_FORWARD_L3_NOLIDAR_RED = 4.9;
// constexpr float MOVE_START_SLIDE_L3_NOLIDAR_RED = -0.6;
// constexpr float MOVE_START_YAW_L3_NOLIDAR_RED = 0.0;
// constexpr float MOVE_START_SPEED_L3_NOLIDAR_RED = 1.5;

constexpr float MOVE_SETUP_FORWARD_L3_RED = -1.0;
constexpr float MOVE_SETUP_SLIDE_L3_RED = 0.0;
constexpr float MOVE_SETUP_YAW_L3_RED = 0.0;
constexpr float MOVE_SETUP_SPEED_L3_RED = 1.5;

constexpr float MOVE_STANDBY_FORWARD_L3_RED = -1.0;
constexpr float MOVE_STANDBY_SLIDE_L3_RED = -7.49;
constexpr float MOVE_STANDBY_YAW_L3_RED = 0.0;
constexpr float MOVE_STANDBY_SPEED_L3_RED = 1.5;

constexpr float MOVE_RAMP_FORWARD_L3_RED = 10.0;
constexpr float MOVE_RAMP_SLIDE_L3_RED = -7.49;
constexpr float MOVE_RAMP_YAW_L3_RED = 0.0;
constexpr float MOVE_RAMP_SPEED_L3_RED = 1.5;

//-----------------------------------------

//GLOBAL
constexpr int32_t ARM_WALK_FRONT_DEG = 130;
constexpr int32_t ARM_WALK_BACK_DEG = 70;

constexpr int32_t ARM_PLACE_FRONT_LV2_DEG = 80;
constexpr int32_t ARM_PLACE_BACK_LV2_DEG = 120;

constexpr int32_t ARM_PUSH_FRONT_LV2_DEG = 70;
constexpr int32_t ARM_PUSH_BACK_LV2_DEG = 130;

constexpr int32_t ARM_WAIT_PLACE_FRONT_LV3_DEG = 110;
constexpr int32_t ARM_WAIT_PLACE_BACK_LV3_DEG = 90;

constexpr int32_t ARM_PLACE_FRONT_LV3_DEG = 80;
constexpr int32_t ARM_PLACE_BACK_LV3_DEG = 120;

constexpr int32_t ARM_PUSH_FRONT_LV3_DEG = 75;
constexpr int32_t ARM_PUSH_BACK_LV3_DEG = 125;

constexpr int32_t LIFT_BOX_LV1_FRONT = 1000;
constexpr int32_t LIFT_BOX_LV1_BACK = 1100;

constexpr int32_t LIFT_BOX_LV2_FRONT = 2000;
constexpr int32_t LIFT_BOX_LV2_BACK = 2100;

constexpr int32_t LIFT_BOX_LV3_FRONT = 3800;
constexpr int32_t LIFT_BOX_LV3_BACK = 3900;

// FLED BLUE ROW1
constexpr float MOVE_SETUP_FORWARD_R1_BLUE = 1.5;
constexpr float MOVE_SETUP_SLIDE_R1_BLUE = 0.0;
constexpr float MOVE_SETUP_YAW_R1_BLUE = 0.0;
constexpr float MOVE_SETUP_SPEED_R1_BLUE = 1.5;

constexpr float MOVE_STANDBY_FORWARD_R1_BLUE = 1.5;
constexpr float MOVE_STANDBY_SLIDE_R1_BLUE = 0.0;
constexpr float MOVE_STANDBY_YAW_R1_BLUE = 0.0;
constexpr float MOVE_STANDBY_SPEED_R1_BLUE = 1.5;

constexpr float MOVE_GO_FORWARD_R1_BLUE = 4.5;
constexpr float MOVE_GO_SLIDE_R1_BLUE = 0.0;
constexpr float MOVE_GO_YAW_R1_BLUE = 0.0;
constexpr float MOVE_GO_SPEED_R1_BLUE = 1.5;

constexpr float MOVE_AFTER_GO_FORWARD_R1_BLUE = 4.865;
constexpr float MOVE_AFTER_GO_SLIDE_R1_BLUE = 0.0;
constexpr float MOVE_AFTER_GO_YAW_R1_BLUE = 0.0;
constexpr float MOVE_AFTER_GO_SPEED_R1_BLUE = 1.0;

constexpr float MOVE_BACKWARD_FORWARD_R1_BLUE = 4.5;
constexpr float MOVE_BACKWARD_SLIDE_R1_BLUE = 0.0;
constexpr float MOVE_BACKWARD_YAW_R1_BLUE = 0.0;
constexpr float MOVE_BACKWARD_SPEED_R1_BLUE = 1.0;

constexpr float MOVE_LV3_FORWARD_R1_BLUE = 4.7;
constexpr float MOVE_LV3_SLIDE_R1_BLUE = 0.0;
constexpr float MOVE_LV3_YAW_R1_BLUE = 0.0;
constexpr float MOVE_LV3_SPEED_R1_BLUE = 1.0;

// FLED BLUE ROW2
constexpr float MOVE_SETUP_FORWARD_R2_BLUE = 1.5;
constexpr float MOVE_SETUP_SLIDE_R2_BLUE = 0.0;
constexpr float MOVE_SETUP_YAW_R2_BLUE = 0.0;
constexpr float MOVE_SETUP_SPEED_R2_BLUE = 1.5;

constexpr float MOVE_STANDBY_FORWARD_R2_BLUE = 1.5;
constexpr float MOVE_STANDBY_SLIDE_R2_BLUE = -0.54;
constexpr float MOVE_STANDBY_YAW_R2_BLUE = 0.0;
constexpr float MOVE_STANDBY_SPEED_R2_BLUE = 1.5;

constexpr float MOVE_GO_FORWARD_R2_BLUE = 4.5;
constexpr float MOVE_GO_SLIDE_R2_BLUE = -0.54;
constexpr float MOVE_GO_YAW_R2_BLUE = 0.0;
constexpr float MOVE_GO_SPEED_R2_BLUE = 1.5;

constexpr float MOVE_AFTER_GO_FORWARD_R2_BLUE = 4.865;
constexpr float MOVE_AFTER_GO_SLIDE_R2_BLUE = -0.54;
constexpr float MOVE_AFTER_GO_YAW_R2_BLUE = 0.0;
constexpr float MOVE_AFTER_GO_SPEED_R2_BLUE = 1.0;

constexpr float MOVE_BACKWARD_FORWARD_R2_BLUE = 4.5;
constexpr float MOVE_BACKWARD_SLIDE_R2_BLUE = -0.54;
constexpr float MOVE_BACKWARD_YAW_R2_BLUE = 0.0;
constexpr float MOVE_BACKWARD_SPEED_R2_BLUE = 1.0;

constexpr float MOVE_LV3_FORWARD_R2_BLUE = 4.7;
constexpr float MOVE_LV3_SLIDE_R2_BLUE = -0.54;
constexpr float MOVE_LV3_YAW_R2_BLUE = 0.0;
constexpr float MOVE_LV3_SPEED_R2_BLUE = 1.0;

// FLED BLUE ROW3
constexpr float MOVE_SETUP_FORWARD_R3_BLUE = 1.5;
constexpr float MOVE_SETUP_SLIDE_R3_BLUE = 0.0;
constexpr float MOVE_SETUP_YAW_R3_BLUE = 0.0;
constexpr float MOVE_SETUP_SPEED_R3_BLUE = 1.5;

constexpr float MOVE_STANDBY_FORWARD_R3_BLUE = 1.5;
constexpr float MOVE_STANDBY_SLIDE_R3_BLUE = -1.08;
constexpr float MOVE_STANDBY_YAW_R3_BLUE = 0.0;
constexpr float MOVE_STANDBY_SPEED_R3_BLUE = 1.5;

constexpr float MOVE_GO_FORWARD_R3_BLUE = 4.5;
constexpr float MOVE_GO_SLIDE_R3_BLUE = -1.08;
constexpr float MOVE_GO_YAW_R3_BLUE = 0.0;
constexpr float MOVE_GO_SPEED_R3_BLUE = 1.5;

constexpr float MOVE_AFTER_GO_FORWARD_R3_BLUE = 4.865;
constexpr float MOVE_AFTER_GO_SLIDE_R3_BLUE = -1.08;
constexpr float MOVE_AFTER_GO_YAW_R3_BLUE = 0.0;
constexpr float MOVE_AFTER_GO_SPEED_R3_BLUE = 1.0;

constexpr float MOVE_BACKWARD_FORWARD_R3_BLUE = 4.5;
constexpr float MOVE_BACKWARD_SLIDE_R3_BLUE = -1.08;
constexpr float MOVE_BACKWARD_YAW_R3_BLUE = 0.0;
constexpr float MOVE_BACKWARD_SPEED_R3_BLUE = 1.0;

constexpr float MOVE_LV3_FORWARD_R3_BLUE = 4.7;
constexpr float MOVE_LV3_SLIDE_R3_BLUE = -1.08;
constexpr float MOVE_LV3_YAW_R3_BLUE = 0.0;
constexpr float MOVE_LV3_SPEED_R3_BLUE = 1.0;

// FLED RED ROW1
constexpr float MOVE_SETUP_FORWARD_R1_RED = 1.5;
constexpr float MOVE_SETUP_SLIDE_R1_RED = 0.0;
constexpr float MOVE_SETUP_YAW_R1_RED = 0.0;
constexpr float MOVE_SETUP_SPEED_R1_RED = 1.5;

constexpr float MOVE_STANDBY_FORWARD_R1_RED = 1.5;
constexpr float MOVE_STANDBY_SLIDE_R1_RED = 0.0;
constexpr float MOVE_STANDBY_YAW_R1_RED = 0.0;
constexpr float MOVE_STANDBY_SPEED_R1_RED = 1.5;

constexpr float MOVE_GO_FORWARD_R1_RED = 4.5;
constexpr float MOVE_GO_SLIDE_R1_RED = 0.0;
constexpr float MOVE_GO_YAW_R1_RED = 0.0;
constexpr float MOVE_GO_SPEED_R1_RED = 1.5;

constexpr float MOVE_AFTER_GO_FORWARD_R1_RED = 4.865;
constexpr float MOVE_AFTER_GO_SLIDE_R1_RED = 0.0;
constexpr float MOVE_AFTER_GO_YAW_R1_RED = 0.0;
constexpr float MOVE_AFTER_GO_SPEED_R1_RED = 1.0;

constexpr float MOVE_BACKWARD_FORWARD_R1_RED = 4.5;
constexpr float MOVE_BACKWARD_SLIDE_R1_RED = 0.0;
constexpr float MOVE_BACKWARD_YAW_R1_RED = 0.0;
constexpr float MOVE_BACKWARD_SPEED_R1_RED = 1.0;

constexpr float MOVE_LV3_FORWARD_R1_RED = 4.7;
constexpr float MOVE_LV3_SLIDE_R1_RED = 0.0;
constexpr float MOVE_LV3_YAW_R1_RED = 0.0;
constexpr float MOVE_LV3_SPEED_R1_RED = 1.0;

// FLED RED ROW2
constexpr float MOVE_SETUP_FORWARD_R2_RED = 1.5;
constexpr float MOVE_SETUP_SLIDE_R2_RED = 0.0;
constexpr float MOVE_SETUP_YAW_R2_RED = 0.0;
constexpr float MOVE_SETUP_SPEED_R2_RED = 1.5;

constexpr float MOVE_STANDBY_FORWARD_R2_RED = 1.5;
constexpr float MOVE_STANDBY_SLIDE_R2_RED = 0.54;
constexpr float MOVE_STANDBY_YAW_R2_RED = 0.0;
constexpr float MOVE_STANDBY_SPEED_R2_RED = 1.5;

constexpr float MOVE_GO_FORWARD_R2_RED = 4.5;
constexpr float MOVE_GO_SLIDE_R2_RED = 0.54;
constexpr float MOVE_GO_YAW_R2_RED = 0.0;
constexpr float MOVE_GO_SPEED_R2_RED = 1.5;

constexpr float MOVE_AFTER_GO_FORWARD_R2_RED = 4.865;
constexpr float MOVE_AFTER_GO_SLIDE_R2_RED = 0.54;
constexpr float MOVE_AFTER_GO_YAW_R2_RED = 0.0;
constexpr float MOVE_AFTER_GO_SPEED_R2_RED = 1.0;

constexpr float MOVE_BACKWARD_FORWARD_R2_RED = 4.5;
constexpr float MOVE_BACKWARD_SLIDE_R2_RED = 0.54;
constexpr float MOVE_BACKWARD_YAW_R2_RED = 0.0;
constexpr float MOVE_BACKWARD_SPEED_R2_RED = 1.0;

constexpr float MOVE_LV3_FORWARD_R2_RED = 4.7;
constexpr float MOVE_LV3_SLIDE_R2_RED = 0.54;
constexpr float MOVE_LV3_YAW_R2_RED = 0.0;
constexpr float MOVE_LV3_SPEED_R2_RED = 1.0;

// FLED RED ROW3
constexpr float MOVE_SETUP_FORWARD_R3_RED = 1.5;
constexpr float MOVE_SETUP_SLIDE_R3_RED = 0.0;
constexpr float MOVE_SETUP_YAW_R3_RED = 0.0;
constexpr float MOVE_SETUP_SPEED_R3_RED = 1.5;

constexpr float MOVE_STANDBY_FORWARD_R3_RED = 1.5;
constexpr float MOVE_STANDBY_SLIDE_R3_RED = 1.08;
constexpr float MOVE_STANDBY_YAW_R3_RED = 0.0;
constexpr float MOVE_STANDBY_SPEED_R3_RED = 1.5;

constexpr float MOVE_GO_FORWARD_R3_RED = 4.5;
constexpr float MOVE_GO_SLIDE_R3_RED = 1.08;
constexpr float MOVE_GO_YAW_R3_RED = 0.0;
constexpr float MOVE_GO_SPEED_R3_RED = 1.5;

constexpr float MOVE_AFTER_GO_FORWARD_R3_RED = 4.865;
constexpr float MOVE_AFTER_GO_SLIDE_R3_RED = 1.08;
constexpr float MOVE_AFTER_GO_YAW_R3_RED = 0.0;
constexpr float MOVE_AFTER_GO_SPEED_R3_RED = 1.0;

constexpr float MOVE_BACKWARD_FORWARD_R3_RED = 4.5;
constexpr float MOVE_BACKWARD_SLIDE_R3_RED = 1.08;
constexpr float MOVE_BACKWARD_YAW_R3_RED = 0.0;
constexpr float MOVE_BACKWARD_SPEED_R3_RED = 1.0;

constexpr float MOVE_LV3_FORWARD_R3_RED = 4.7;
constexpr float MOVE_LV3_SLIDE_R3_RED = 1.08;
constexpr float MOVE_LV3_YAW_R3_RED = 0.0;
constexpr float MOVE_LV3_SPEED_R3_RED = 1.0;