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
