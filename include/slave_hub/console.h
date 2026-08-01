#pragma once

#include "config.h"

// One text command parser shared by the USB Serial Monitor and by Serial1.
// Commands coming from the Master get an "ACK,<command>" reply.

void printHelp();
void processCommand(char *command, bool fromMaster = false);
void handleCommands();
