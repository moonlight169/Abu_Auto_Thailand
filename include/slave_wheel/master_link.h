#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial1 protocol (PA9 TX / PA10 RX, 115200) with the Teensy Master.
//
//  Master -> this board
//    T,FL,FR,RL,RR\n   target speed of each wheel in RPM
//    S\n               stop immediately
//    Q\n               request one feedback packet
//
//  this board -> Master
//    F,time,FL_count,FR_count,RL_count,RR_count,FL_rpm,FR_rpm,RL_rpm,RR_rpm\n
// ---------------------------------------------------------------------------

extern bool serial1ControlActive;

void sendSerial1Feedback();
void readSerial1Commands();
