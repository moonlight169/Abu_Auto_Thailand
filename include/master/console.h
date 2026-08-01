#pragma once

#include "config.h"

// USB Serial Monitor command console. See printHelp() for the full list.

void printHelp();
void processUsbLine(char *line);
void readUsbSerial();
