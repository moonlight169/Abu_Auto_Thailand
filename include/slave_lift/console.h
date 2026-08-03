#pragma once

#include "config.h"

// USB Serial Monitor test interface. The Master link feeds the very same
// executeCommand(), so anything listed by printHelp() also works over
// Serial1. Gains are typed as integers x1000, e.g. `5300fp` = front Kp 5.300.

void printHelp();
void printPose();
void printSettings();
void printLimits();

void executeCommand(char *command);
void readSerialCommands();
