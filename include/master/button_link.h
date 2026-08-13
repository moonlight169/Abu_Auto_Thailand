#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial3 @ BUTTON_BAUD to the keypad panel (Blue Pill F103).
//
//  Keypad -> Master: one mission code as plain ASCII, terminated by '\n',
//  repeated 5 times 20 ms apart. Nothing is sent back yet, although the TX3
//  wire is already there.
//
//  What arrives is a code, not a command: the Master never decodes what the
//  digits mean. It filters the line, votes on the repeats and hands the
//  winning string to startMissionByCode(), which matches it against the name
//  of a program in mission_list.cpp.
//
//    layer 1  empty line            -> dropped silently
//    layer 2  digits 0-9 only       -> BAD CODE (NOT DIGITS)
//    layer 3  length matches Mode   -> BAD CODE (LENGTH) / (UNKNOWN MODE)
//    layer 4  found in the registry -> UNKNOWN MISSION CODE
//
//  Layers 1-3 live here on purpose. They must NOT move into
//  startMissionByCode(), because the Hub buttons call that with names like
//  "MISSION 1", which is neither all digits nor 5/7 characters long.
// ---------------------------------------------------------------------------

void beginButtonSerial();

// Reads whatever Serial3 has buffered and closes the vote window when it
// expires. Call it once per loop(), before updateMission().
void readButtonSerial();

// Runs one code through layers 1-3 and adds it to the vote window.
// `source` only labels the log line ("SERIAL3", "CONSOLE"), so the USB test
// command can reuse the exact same path as the real keypad.
void submitMissionCode(const char *code, const char *source);
