#pragma once

#include "config.h"

// Debounced view of the four end-stop switches. The state machine uses the
// STABLE state; motor.cpp still checks the raw pin for instant protection.

extern DebouncedLimit topFrontLimit;
extern DebouncedLimit topBackLimit;
extern DebouncedLimit bottomFrontLimit;
extern DebouncedLimit bottomBackLimit;

void updateDebouncedLimits();
