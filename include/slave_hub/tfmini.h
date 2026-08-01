#pragma once

#include "config.h"

// Four TFMini-S range finders, one per UART. Distances are in centimetres.

extern TfminiData tfminiData[TFMINI_COUNT];

void beginTfmini();
void updateTfmini();

bool tfminiFresh(uint8_t index);
uint8_t getTfminiValidMask();

void printTfminiMonitor();
