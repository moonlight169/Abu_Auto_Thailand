#include "master_link.h"

#include "arm_control.h"
#include "motor.h"

MasterJob masterJob = MASTER_JOB_NONE;
float masterBottomTargetDeg = 0.0f;
float masterTopTargetDeg = 0.0f;

uint32_t activeSequence = 0;
uint32_t lastFinishedSequence = 0;
char activeCommand[12] = "";
char lastFinishedReply[MASTER_REPLY_BUFFER_SIZE] = "";

static char masterRxBuffer[MASTER_RX_BUFFER_SIZE];
static size_t masterRxLength = 0;
static bool masterRxOverflow = false;

void sendMasterLine(const char *line) {
  Serial1.println(line);
}

void rememberAndSendFinished(const char *line) {
  strncpy(lastFinishedReply, line, sizeof(lastFinishedReply) - 1);
  lastFinishedReply[sizeof(lastFinishedReply) - 1] = '\0';
  lastFinishedSequence = activeSequence;
  sendMasterLine(line);
  activeSequence = 0;
  activeCommand[0] = '\0';
}

void masterSendError(uint32_t sequence, const char *command,
                     const char *message, bool finishJob) {
  char reply[MASTER_REPLY_BUFFER_SIZE];
  snprintf(reply, sizeof(reply), "ERR,%lu,%s,%s",
           (unsigned long)sequence, command, message);
  if (finishJob && sequence == activeSequence) {
    rememberAndSendFinished(reply);
  } else {
    sendMasterLine(reply);
  }
}

bool parseCsvFloat(const String &field, float &value) {
  String text = field;
  text.trim();
  if (text.length() == 0 || text.length() >= 24) return false;

  char buffer[24];
  text.toCharArray(buffer, sizeof(buffer));
  char *end = nullptr;
  value = strtof(buffer, &end);
  return end != buffer && *end == '\0' && isfinite(value);
}

void masterSendStatus(uint32_t sequence) {
  Serial1.print(F("STATUS,"));
  Serial1.print(sequence);
  Serial1.print(',');
  Serial1.print(stateName());
  Serial1.print(',');
  Serial1.print(bottomHomed ? 1 : 0);
  Serial1.print(',');
  Serial1.print(topHomed ? 1 : 0);
  Serial1.print(',');
  Serial1.print(getBottomPositionDeg(), 2);
  Serial1.print(',');
  Serial1.print(getTopPositionDeg(), 2);
  Serial1.print(',');
  Serial1.print(bottomHolding ? 1 : 0);
  Serial1.print(',');
  Serial1.print(topHolding ? 1 : 0);
  Serial1.print(',');
  Serial1.print(isPressed(BOTTOM_LIMIT_FRONT) ? 1 : 0);
  Serial1.print(',');
  Serial1.print(isPressed(BOTTOM_LIMIT_BACK) ? 1 : 0);
  Serial1.print(',');
  Serial1.print(isPressed(TOP_LIMIT_FRONT) ? 1 : 0);
  Serial1.print(',');
  Serial1.println(isPressed(TOP_LIMIT_BACK) ? 1 : 0);
}

void serviceMasterJob() {
  char reply[MASTER_REPLY_BUFFER_SIZE];
  switch (masterJob) {
    case MASTER_JOB_HOME:
      if (topHomed && state == TOP_POSITION_MOVE && topHolding) {
        masterJob = MASTER_JOB_NONE;
        snprintf(reply, sizeof(reply), "DONE,%lu,HOME,%.2f,%.2f",
                 (unsigned long)activeSequence,
                 getBottomPositionDeg(), getTopPositionDeg());
        rememberAndSendFinished(reply);
      }
      break;

    case MASTER_JOB_BOTTOM:
      if (state == BOTTOM_POSITION_MOVE && bottomHolding) {
        masterJob = MASTER_JOB_NONE;
        snprintf(reply, sizeof(reply), "DONE,%lu,B,%.2f",
                 (unsigned long)activeSequence, getBottomPositionDeg());
        rememberAndSendFinished(reply);
      }
      break;

    case MASTER_JOB_TOP:
      if (state == TOP_POSITION_MOVE && topHolding) {
        masterJob = MASTER_JOB_NONE;
        snprintf(reply, sizeof(reply), "DONE,%lu,T,%.2f",
                 (unsigned long)activeSequence, getTopPositionDeg());
        rememberAndSendFinished(reply);
      }
      break;

    case MASTER_JOB_POSE_BOTTOM:
      if (state == BOTTOM_POSITION_MOVE && bottomHolding) {
        masterJob = MASTER_JOB_POSE_TOP;
        if (!commandTopPosition(masterTopTargetDeg)) {
          masterJob = MASTER_JOB_NONE;
          masterSendError(activeSequence, activeCommand,
                          "POSE_TOP_REJECTED", true);
        }
      }
      break;

    case MASTER_JOB_POSE_TOP:
      if (state == TOP_POSITION_MOVE && topHolding) {
        masterJob = MASTER_JOB_NONE;
        snprintf(reply, sizeof(reply), "DONE,%lu,POS,%.2f,%.2f",
                 (unsigned long)activeSequence,
                 getBottomPositionDeg(), getTopPositionDeg());
        rememberAndSendFinished(reply);
      }
      break;

    case MASTER_JOB_NONE:
      break;
  }
}

bool masterIsBusy() {
  return masterJob != MASTER_JOB_NONE ||
         (state != IDLE && state != FAULT &&
          !((state == BOTTOM_POSITION_MOVE && bottomHolding) ||
            (state == TOP_POSITION_MOVE && topHolding)));
}

static bool parseUnsigned(const char *text, uint32_t &value) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (*end != '\0') return false;
  value = (uint32_t)parsed;
  return value != 0;
}

static bool parseFloatField(const char *text, float &value) {
  if (text == nullptr || *text == '\0') return false;
  char *end = nullptr;
  value = strtof(text, &end);
  return *end == '\0' && isfinite(value);
}

static void sendAck(uint32_t sequence, const char *command) {
  Serial1.print(F("ACK,"));
  Serial1.print(sequence);
  Serial1.print(',');
  Serial1.println(command);
}

static void beginMasterJob(uint32_t sequence, const char *command,
                           MasterJob job) {
  activeSequence = sequence;
  strncpy(activeCommand, command, sizeof(activeCommand) - 1);
  activeCommand[sizeof(activeCommand) - 1] = '\0';
  masterJob = job;
  sendAck(sequence, command);
}

static void processMasterCommand(char *line) {
  char *save = nullptr;
  char *prefix = strtok_r(line, ",", &save);
  char *sequenceText = strtok_r(nullptr, ",", &save);
  char *command = strtok_r(nullptr, ",", &save);

  if (!prefix || strcmp(prefix, "CMD") != 0 || !command) {
    masterSendError(0, "PARSE", "FORMAT_CMD_SEQ_COMMAND");
    return;
  }

  uint32_t sequence = 0;
  if (!parseUnsigned(sequenceText, sequence)) {
    masterSendError(0, command, "BAD_SEQUENCE");
    return;
  }

  // A retry with the same sequence never starts physical motion again.
  if (sequence == activeSequence && activeSequence != 0) {
    if (strcmp(command, activeCommand) == 0) {
      sendAck(sequence, activeCommand);
    } else {
      masterSendError(sequence, command, "SEQUENCE_CONFLICT");
    }
    return;
  }
  if (sequence == lastFinishedSequence && lastFinishedReply[0] != '\0') {
    sendMasterLine(lastFinishedReply);
    return;
  }
  // Do not reject a numerically smaller sequence here.  The Master may reboot
  // independently and legitimately restart its counter at 1.  Duplicate
  // protection is still provided above for the active and last-finished IDs.

  if (strcmp(command, "STATUS") == 0) {
    masterSendStatus(sequence);
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    activeSequence = sequence;
    strncpy(activeCommand, "STOP", sizeof(activeCommand));
    stopAll();
    activeSequence = sequence;
    strncpy(activeCommand, "STOP", sizeof(activeCommand));
    char reply[48];
    snprintf(reply, sizeof(reply), "DONE,%lu,STOP", (unsigned long)sequence);
    rememberAndSendFinished(reply);
    return;
  }

  if (strcmp(command, "CLEAR") == 0) {
    bottomMotor(0);
    topMotor(0);
    state = IDLE;
    masterJob = MASTER_JOB_NONE;
    activeSequence = sequence;
    strncpy(activeCommand, "CLEAR", sizeof(activeCommand));
    char reply[48];
    snprintf(reply, sizeof(reply), "DONE,%lu,CLEAR", (unsigned long)sequence);
    rememberAndSendFinished(reply);
    return;
  }

  if (state == FAULT) {
    masterSendError(sequence, command, "FAULT_ACTIVE");
    return;
  }
  if (masterIsBusy()) {
    masterSendError(sequence, command, "BUSY");
    return;
  }

  if (strcmp(command, "HOME") == 0) {
    beginMasterJob(sequence, command, MASTER_JOB_HOME);
    startHome();
    return;
  }

  if (strcmp(command, "B") == 0 || strcmp(command, "T") == 0) {
    char *valueText = strtok_r(nullptr, ",", &save);
    if (strtok_r(nullptr, ",", &save) != nullptr) {
      masterSendError(sequence, command, "TOO_MANY_FIELDS");
      return;
    }
    float degrees = 0.0f;
    if (!parseFloatField(valueText, degrees)) {
      masterSendError(sequence, command, "BAD_POSITION");
      return;
    }
    if (strcmp(command, "B") == 0) {
      if (!bottomHomed) {
        masterSendError(sequence, command, "BOTTOM_NOT_HOMED");
        return;
      }
      beginMasterJob(sequence, command, MASTER_JOB_BOTTOM);
      commandBottomPosition(degrees);
    } else {
      if (!topHomed) {
        masterSendError(sequence, command, "TOP_NOT_HOMED");
        return;
      }
      beginMasterJob(sequence, command, MASTER_JOB_TOP);
      commandTopPosition(degrees);
    }
    return;
  }

  if (strcmp(command, "POS") == 0) {
    char *bottomText = strtok_r(nullptr, ",", &save);
    char *topText = strtok_r(nullptr, ",", &save);
    float bottomDeg = 0.0f;
    float topDeg = 0.0f;
    if (strtok_r(nullptr, ",", &save) != nullptr ||
        !parseFloatField(bottomText, bottomDeg) ||
        !parseFloatField(topText, topDeg)) {
      masterSendError(sequence, command, "BAD_POSITION");
      return;
    }
    if (!bottomHomed || !topHomed) {
      masterSendError(sequence, command, "NOT_HOMED");
      return;
    }
    masterBottomTargetDeg =
        constrain(bottomDeg, BOTTOM_MIN_DEG, BOTTOM_MAX_DEG);
    masterTopTargetDeg =
        constrain(topDeg, TOP_MIN_DEG, TOP_MAX_DEG);
    beginMasterJob(sequence, command, MASTER_JOB_POSE_BOTTOM);
    commandBottomPosition(masterBottomTargetDeg);
    return;
  }

  masterSendError(sequence, command, "UNKNOWN_COMMAND");
}

void readMasterCommands() {
  while (Serial1.available()) {
    const char c = (char)Serial1.read();
    if (c == '\r' || c == '\n') {
      if (masterRxOverflow) {
        masterSendError(0, "PARSE", "LINE_TOO_LONG");
      } else if (masterRxLength > 0) {
        masterRxBuffer[masterRxLength] = '\0';
        processMasterCommand(masterRxBuffer);
      }
      masterRxLength = 0;
      masterRxOverflow = false;
    } else if (!masterRxOverflow &&
               masterRxLength < sizeof(masterRxBuffer) - 1) {
      // Protocol keywords are uppercase; normalise input without String.
      masterRxBuffer[masterRxLength++] =
          (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
    } else {
      // Discard everything until newline, so a broken frame cannot become
      // a valid command fragment.
      masterRxOverflow = true;
    }
  }
}
