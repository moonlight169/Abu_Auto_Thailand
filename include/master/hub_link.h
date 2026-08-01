#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial1 @ 460800 to the sensor hub (Teensy 4.1).
//
//  Hub -> Master, 20 Hz:
//    HUB,<ms>,<BOX|WALL>,<found>,<dist>,<offset>,<angle>,<width>,<points>,
//        <inputMask>,<relayMask>,<arm>,<spin>,<tf1>,<tf2>,<tf3>,<tf4>,<tfValid>
//    Shorter legacy layouts (13 / 17 fields, or the 4..11 field "short
//    format") are still accepted.
//
//  Master -> Hub: text commands, e.g. "R1 ON", "ARM 40", "MODE BOX",
//                 "LIDAR ON", "STREAM ON".
// ---------------------------------------------------------------------------

extern HubState hub;

void beginHubSerial();
void readHubSerial();
bool parseHubPacket(char *line);

void setHubRelay(uint8_t relayNumber, bool on);
void setHubArm(uint8_t angle);
void setHubSpin(uint8_t angle);

bool hubInputActive(uint8_t bit);
bool tfDistanceInRange(uint8_t tfIndex,
                       uint16_t minDistanceCm,
                       uint16_t maxDistanceCm);
bool hubLaserDetected(uint8_t laserBit);

bool tf2ForwardStopDetected();
bool tf1ForwardStopDetected();
bool tf1DescendFloorDetected();

void printHubStatus();
