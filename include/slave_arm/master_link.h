#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  CSV protocol with the Teensy Master on Serial1 (PA9 TX / PA10 RX).
//
//  Master -> Arm    CMD,<seq>,<command>[,<args>]
//                   commands: HOME | B,<deg> | T,<deg> | POS,<b>,<t>
//                             STATUS | STOP | CLEAR
//  Arm -> Master    ACK,<seq>,<command>
//                   STATE,<seq>,<phase>
//                   DONE,<seq>,<command>[,<values>]
//                   ERR,<seq>,<command>,<reason>
//                   STATUS,<seq>,...
//
//  A retry with the same sequence never starts physical motion twice.
// ---------------------------------------------------------------------------

extern MasterJob masterJob;
extern float masterBottomTargetDeg;
extern float masterTopTargetDeg;

extern uint32_t activeSequence;
extern uint32_t lastFinishedSequence;
extern char activeCommand[12];
extern char lastFinishedReply[MASTER_REPLY_BUFFER_SIZE];

void sendMasterLine(const char *line);
void rememberAndSendFinished(const char *line);
void masterSendError(uint32_t sequence, const char *command,
                     const char *message, bool finishJob = false);
void masterSendStatus(uint32_t sequence = 0);

bool parseCsvFloat(const String &field, float &value);

bool masterIsBusy();
void serviceMasterJob();
void readMasterCommands();
