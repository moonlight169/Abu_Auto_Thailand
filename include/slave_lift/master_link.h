#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Line protocol with the Teensy Master on Serial1 (PA9 TX / PA10 RX).
//
//  Master -> lift   LP,<frontPulse>,<backPulse>    move to a pulse target
//                   LZ | HOME                      home onto the BOTTOM stops
//                   S                              stop both columns
//                   (every console command is accepted on this link as well)
//  lift -> Master   LIFT_POS,<front>,<back>        streamed every 20 ms
//                   LIFT_BUSY | LIFT_REACHED,<front>,<back>
//                   LIFT_HOMING | LIFT_HOME_REACHED
//                   LIFT_ERROR,<reason>
//
//  Everything sent to the Master is echoed to the USB console as [TEENSY].
// ---------------------------------------------------------------------------

void sendMasterStatus(const char *status);
void sendLiftReached(int32_t front, int32_t back);
void sendLiftPosition();

void readMasterCommands();
