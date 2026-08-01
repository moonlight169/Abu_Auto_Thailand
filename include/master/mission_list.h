#pragma once

#include "mission_steps.h"

// Registry of every mission program. The tables themselves are in
// mission_list.cpp -- that is the file to edit when changing a run.

extern const MissionProgram missionPrograms[];
extern const size_t MISSION_PROGRAM_COUNT;

// 0 = Mission 1. Changed with "SEL n" or "Mn" on the console.
extern uint8_t selectedMission;

const MissionProgram &activeMissionProgram();
