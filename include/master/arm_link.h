#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Serial6 @ 115200 to the external Arm slave (Blackpill F411CE).
//
//  Master -> Arm    CMD,<seq>,HOME | B,<deg> | T,<deg> | POS,<b>,<t>
//                   CMD,<seq>,STATUS | STOP | CLEAR
//  Arm -> Master    ACK / STATE / DONE / ERR, each echoing <seq>.
//
//  A command is retransmitted up to ARM_MAX_RETRIES times until it is ACKed;
//  DONE must still arrive before ARM_DEFAULT_TIMEOUT_MS.
// ---------------------------------------------------------------------------

extern ArmTaskStatus armTaskStatus;
extern ArmResetState armResetState;
extern uint32_t armCommandStartMs;
extern bool armOnline;

extern char armExpectedDone[16];
extern char armTxPacket[80];

uint32_t allocateArmSequence();
void sendArmPacket(const char *packet);
void sendArmAuxCommand(const char *command);

void readArmSerial();
void updateArmCommunication();

void startArmCommand(const char *command);
void startArmHome();
void startArmBottom(float degrees);
void startArmTop(float degrees);
void startArmPosition(float bottomDegrees, float topDegrees);

void startArmClearAndHome();
void updateArmClearAndHome();
