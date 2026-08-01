#pragma once

#include "config.h"

// USB Serial Monitor test interface. See printHelp() for the command list.

extern bool limitStreamEnabled;

void printHelp();
void printStatus();
void printLimits();

void readSerialCommands();
void initializeConsole();
