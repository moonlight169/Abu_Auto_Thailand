#pragma once

#include "config.h"

// USB LiDAR diagnostics.
//   0 = off, 1 = summary, 2 = summary + box bar, 3 = summary + CSV points

extern uint8_t MONITOR_MODE;
extern uint32_t MONITOR_INTERVAL_MS;

void printMonitorSummary();
void printMonitorPoints();
void updateMonitor();
