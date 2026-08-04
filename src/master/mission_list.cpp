// ===========================================================================
//  >>> THE MISSION SEQUENCES. THIS IS THE FILE TO EDIT FOR A NEW RUN. <<<
//
//  Each row is one step. The runner starts a row, waits until it reports
//  done, then moves to the next one. See mission_steps.h for the full list
//  of available *_STEP() builders and their arguments.
//
//  Add another mission by writing a new `const MissionStep missionN[]` array
//  and registering it in missionPrograms[] at the bottom.
// ===========================================================================

#include "mission_list.h"

/////////////////////////////////////////////////////////////////////////////////MissionStep/////////////////////////////////////////////////////////////////////////////////

// Add more mission programs by creating another MissionStep array and
// registering it in missionPrograms[] below.
const MissionStep exMission[] =
{
  RESET_STEP(),
  LIFT_STEP(100, 200),
  MOVE_STEP(3.8,0.5,0,20),
  LIFT_STEP(1000, 1100),
  WAIT_STEP(200),
  LIFT_STEP(2000, 2100),
  WAIT_STEP(200),
  LIFT_STEP(3800, 3900),
  WAIT_STEP(200),
  LIDAR_PARK_STEP(385, 30000),
  HUB_RELAY_STEP(1, 1),
  WAIT_STEP(500),
  ARM_BOTTOM_STEP(5),
  ARM_TOP_STEP(25),
  HUB_RELAY_STEP(2, 1),
  WAIT_STEP(500),
  ARM_TOP_STEP(100),
  WAIT_STEP(500),
  ARM_BOTTOM_STEP(180),
  HUB_RELAY_STEP(1, 0),
  WAIT_STEP(500),
  LIFT_STEP(0, 3900),
  LASER3_FORWARD_STEP(0.5,15000),
  LIFT_STEP(0, 0),

  LASER4_FORWARD_STEP(0.5,15000),
  LIFT_STEP(100, 200),

  FRONT_LIMIT_STEP(0.5f, 30000),
  LIFT_STEP(2000, 2100),
  LIDAR_PARK_STEP(385, 30000),
  HUB_RELAY_STEP(1, 1),
  WAIT_STEP(500),
  ARM_TOP_STEP(175),
  HUB_RELAY_STEP(3, 1),
  WAIT_STEP(500),
  ARM_TOP_STEP(100),
  WAIT_STEP(500),
  LIFT_STEP(0, 2100),
  LASER3_FORWARD_STEP(0.5,15000),
  LIFT_STEP(0, 0),
  LASER4_FORWARD_STEP(0.5,15000),
  RESET_STEP(),
  LIFT_STEP(100, 200),
  HUB_RELAY_STEP(1, 0),
  LIDAR_PARK_STEP(385, 30000),
  HUB_RELAY_STEP(1, 1),
  WAIT_STEP(500),
  ARM_BOTTOM_STEP(90),
  HUB_RELAY_STEP(3, 0),
  ARM_BOTTOM_STEP(180),
  ARM_TOP_STEP(180),
  HUB_RELAY_STEP(3, 1),
  WAIT_STEP(500),
  ARM_TOP_STEP(100),
  WAIT_STEP(500),
  LIFT_STEP(0, 50),

  TF34_EDGE_FRONT_LIFT_STEP(0.30f, 1800, 0, 15000),
  TF1_FORWARD_BACK_LIFT_STEP(0.30f, 1800, 15000),
  FORWARD_LZ_BACK_STEP(0.50f, 1500, 0.3f, 2000, 15000),

    LIFT_STEP(0, 50),
  TF34_EDGE_FRONT_LIFT_STEP(0.30f, 1800, 0, 15000),
  TF1_FORWARD_BACK_LIFT_STEP(0.30f, 1800, 15000),
  FORWARD_LZ_BACK_STEP(0.50f, 1500, 0.3f, 2000, 15000),

    LIFT_STEP(0, 50),
  TF34_EDGE_FRONT_LIFT_STEP(0.30f, 1800, 0, 15000),
  TF1_FORWARD_BACK_LIFT_STEP(0.30f, 1800, 15000),
  FORWARD_LZ_BACK_STEP(0.50f, 1500, 0.3f, 2000, 15000),

  END_STEP()
};

const MissionStep mission1[] =
{
  RESET_STEP(),
  HUB_ARM_STEP(5),
  HUB_SPIN_STEP(0),
  HUB_RELAY_STEP(4, 1),
  LIFT_STEP(2300, 2400),
  MOVE_STEP(1.4,-1.0,0,15),
  HUB_ARM_STEP(71), 
  FRONT_LIMIT_STEP(0.4f, 15000),
  RESET_STEP_2(),
  LASER5_PICK_VERIFY_STEP(-0.25f, +0.04f, 71, 15000),
  LIFT_STEP(2550, 2650),
  HUB_ARM_STEP(10),
  WAIT_STEP(1000),
  HUB_SPIN_STEP(133),
  WAIT_STEP(1000),
  LDR1_WAIT_STEP(0),       
  HUB_RELAY_STEP(4, 1),
  END_STEP()
};

const MissionStep mission2[] =
{
  RESET_STEP(),
  HUB_ARM_STEP(5),
  HUB_SPIN_STEP(0),
  HUB_RELAY_STEP(4, 1),
  LIFT_STEP(2300, 2400),
  MOVE_STEP(1.4,0.5,0,15),
  HUB_ARM_STEP(72),
  FRONT_LIMIT_STEP(0.4f, 15000),
  RESET_STEP_2(),
  LASER5_PICK_VERIFY_STEP(0.25f, +0.04f, 72, 15000),
  LIFT_STEP(2550, 2650),
  HUB_ARM_STEP(10),
  WAIT_STEP(1000),
  HUB_SPIN_STEP(133),
  WAIT_STEP(1000),
  LDR1_WAIT_STEP(0),     
  HUB_RELAY_STEP(4, 1),
  END_STEP()
};

/////////////////////////////////////////////////////////////////////////////////MissionStep/////////////////////////////////////////////////////////////////////////////////

const MissionProgram missionPrograms[] =
{
  { "exMission", exMission, ARRAY_COUNT(exMission) },
  { "MISSION 1", mission1, ARRAY_COUNT(mission1) },
  { "MISSION 2", mission2, ARRAY_COUNT(mission2) },
};

const size_t MISSION_PROGRAM_COUNT = ARRAY_COUNT(missionPrograms);

uint8_t selectedMission = 0;  // 0 = Mission 1

const MissionProgram &activeMissionProgram()
{
  return missionPrograms[selectedMission];
}
