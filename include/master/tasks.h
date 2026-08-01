#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Status of the three long-running jobs the mission runner waits on, plus
//  their watchdog timers.
// ---------------------------------------------------------------------------

extern TaskStatus moveTaskStatus;
extern TaskStatus liftTaskStatus;
extern TaskStatus boxParkingTaskStatus;

extern uint32_t moveTaskStartMs;
extern uint32_t moveTaskTimeoutMs;
extern uint32_t liftTaskStartMs;
extern uint32_t liftTaskTimeoutMs;

void updateRobotTasks();
