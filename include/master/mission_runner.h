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

// Starts the program whose `name` matches this string exactly, so callers
// never depend on the position of an entry in missionPrograms[]. Used by the
// keypad codes ("0120011") and by the Hub buttons ("MISSION 1").
//
// Returns false only when the name is not in the registry. A recognised name
// still goes through startMission(), so it is refused there as usual while a
// mission is running -- true means "the name exists", not "it is running now".
//
// NOTE: no digit or length checking happens here. That belongs to whoever
// reads the wire (button_link.cpp), because the Hub buttons pass names that
// are neither numeric nor 5/7 characters long.
bool startMissionByCode(const char *code);

void resetBeforeMission();
void resetBeforeMission2();
