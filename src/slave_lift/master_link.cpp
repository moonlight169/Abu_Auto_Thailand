#include "master_link.h"

#include "console.h"
#include "encoder.h"

static char masterCommandBuffer[COMMAND_BUFFER_SIZE] = {};
static uint8_t masterCommandIndex = 0;

void sendMasterStatus(const char *status) {
  Serial1.println(status);
  Serial.print(F("[TEENSY] "));
  Serial.println(status);
}

void sendLiftReached(int32_t front, int32_t back) {
  Serial1.print(F("LIFT_REACHED,"));
  Serial1.print(front);
  Serial1.print(F(","));
  Serial1.println(back);

  Serial.print(F("[TEENSY] LIFT_REACHED,"));
  Serial.print(front);
  Serial.print(F(","));
  Serial.println(back);
}

void sendLiftPosition() {
  static uint32_t lastSendMs = 0;

  if (millis() - lastSendMs < LIFT_POS_PERIOD_MS) {
    return;
  }

  lastSendMs = millis();

  const int32_t front = readFrontEncoder();
  const int32_t back = readBackEncoder();

  Serial1.print(F("LIFT_POS,"));
  Serial1.print(front);
  Serial1.print(F(","));
  Serial1.println(back);
}

void readMasterCommands() {
  while (Serial1.available() > 0) {
    const char received = (char)Serial1.read();

    if (received == '\r' || received == '\n') {
      if (masterCommandIndex > 0) {
        masterCommandBuffer[masterCommandIndex] = '\0';
        executeCommand(masterCommandBuffer);
        masterCommandIndex = 0;
      }
    } else if (masterCommandIndex < COMMAND_BUFFER_SIZE - 1) {
      masterCommandBuffer[masterCommandIndex++] = received;
    } else {
      masterCommandIndex = 0;
      Serial.println(F("ERR: command too long"));
      Serial1.println(F("LIFT_ERROR,COMMAND_TOO_LONG"));
    }
  }
}
