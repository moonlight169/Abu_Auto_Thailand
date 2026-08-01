#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial1 @ 460800 to the Teensy Master.
//
//  The full hub packet is streamed at 20 Hz:
//    HUB,<ms>,<BOX|WALL>,<found>,<dist>,<offset>,<angle>,<width>,<points>,
//        <inputMask>,<relayMask>,<arm>,<spin>,<tf1>,<tf2>,<tf3>,<tf4>,<tfValid>
//
//  Single-shot replies are also available: GET LIDAR / IO / SERVO / TFMINI.
// ---------------------------------------------------------------------------

extern bool masterStreamEnabled;
extern uint32_t masterStreamIntervalMs;

void sendMasterLidar();
void sendMasterIo();
void sendMasterServo();
void sendMasterTfmini();
void sendMasterStatus();

void updateMasterStream();
