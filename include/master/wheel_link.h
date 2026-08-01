#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial2 @ 115200 to the wheel driver STM32.
//
//  Master -> wheels   T,FL,FR,RL,RR   target RPM
//                     S               immediate stop
//                     Q               request one feedback packet
//  wheels -> Master   F,time,FL_c,FR_c,RL_c,RR_c,FL_rpm,FR_rpm,RL_rpm,RR_rpm
// ---------------------------------------------------------------------------

extern uint32_t lastCommandSendMs;
extern uint32_t lastFeedbackMs;
extern uint32_t goodPacketCount;
extern uint32_t badPacketCount;

void sendTargetPacket();
void readSlaveSerial();

void printFeedback(uint32_t slaveTime);
void printStatus();
