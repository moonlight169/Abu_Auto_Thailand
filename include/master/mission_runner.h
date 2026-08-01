#pragma once

#include "mission_steps.h"

// ---------------------------------------------------------------------------
//  Mission sequence: start one row, wait until done, then next row.
//  The step tables themselves live in mission_list.cpp.
// ---------------------------------------------------------------------------

extern bool missionRunning;
extern bool missionStepStarted;
extern size_t missionIndex;

bool selectMission(uint8_t missionNumber);
void startMission(uint8_t missionNumber = 0);
void stopMission(const char *reason);
void updateMission();

void resetBeforeMission();
void resetBeforeMission2();
