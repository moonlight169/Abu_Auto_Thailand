#include "console.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "encoder.h"
#include "lift_control.h"
#include "motor.h"
#include "position_pid.h"

static char usbCommandBuffer[COMMAND_BUFFER_SIZE] = {};
static uint8_t usbCommandIndex = 0;

static bool parseTwoLongs(const char *text, long &first, long &second) {
  char *end = nullptr;
  first = strtol(text, &end, 10);
  if (end == text || *end != ',') return false;

  const char *secondText = end + 1;
  second = strtol(secondText, &end, 10);
  while (*end == ' ' || *end == '\t') end++;
  return end != secondText && *end == '\0';
}

void printHelp() {
  Serial.println(F("STM32 LIFT POSITION PID + SOFT DOWN READY"));
  Serial.println(F("Teensy UART: Serial1 115200, TX=PA9, RX=PA10"));
  Serial.println(F("LP,f,b | LZ/HOME | S | Z | POSE | LIMIT | PID"));
  Serial.println(F("PID: 1300p 1500i 0d"));
  Serial.println(F("Separate: 1300fp 1500fi 0fd | 1300bp 1500bi 0bd"));

  Serial.print(F("Soft down: front max="));
  Serial.print(FRONT_DOWN_PWM_MAX);
  Serial.print(F(", back max="));
  Serial.print(BACK_DOWN_PWM_MAX);
  Serial.print(F(", accel="));
  Serial.print(DOWN_ACCEL_STEP);
  Serial.print(F(", decel="));
  Serial.print(DOWN_DECEL_STEP);
  Serial.print(F(", brake="));
  Serial.print(DOWN_BRAKE_ZONE_PULSE);
  Serial.println(F(" pulse"));
}

void printPose() {
  const int32_t front = readFrontEncoder();
  const int32_t back = readBackEncoder();
  const float frontMM =
      (float)front * FRONT_STROKE_MM / FRONT_PULSE_MAX;
  const float backMM =
      (float)back * BACK_STROKE_MM / BACK_PULSE_MAX;

  Serial.print(F("Fmm="));
  Serial.print(frontMM, 1);
  Serial.print(F(",Bmm="));
  Serial.print(backMM, 1);
  Serial.print(F(",Fpos="));
  Serial.print(front);
  Serial.print(F(",Bpos="));
  Serial.print(back);
  Serial.print(F(",targetF="));
  Serial.print(targetFront);
  Serial.print(F(",targetB="));
  Serial.print(targetBack);
  Serial.print(F(",setF="));
  Serial.print(setpointFront, 0);
  Serial.print(F(",setB="));
  Serial.print(setpointBack, 0);
  Serial.print(F(",Fpwm="));
  Serial.print(frontPWM);
  Serial.print(F(",Bpwm="));
  Serial.print(backPWM);
  Serial.print(F(",FT="));
  Serial.print(limitActive(FRONT_LIMIT_TOP));
  Serial.print(F(",FB="));
  Serial.print(limitActive(FRONT_LIMIT_BOTTOM));
  Serial.print(F(",BT="));
  Serial.print(limitActive(BACK_LIMIT_TOP));
  Serial.print(F(",BB="));
  Serial.println(limitActive(BACK_LIMIT_BOTTOM));
}

void printSettings() {
  Serial.print(F("Front Kp="));
  Serial.print(frontKp, 3);
  Serial.print(F(",Ki="));
  Serial.print(frontKi, 3);
  Serial.print(F(",Kd="));
  Serial.print(frontKd, 3);
  Serial.print(F(" | Back Kp="));
  Serial.print(backKp, 3);
  Serial.print(F(",Ki="));
  Serial.print(backKi, 3);
  Serial.print(F(",Kd="));
  Serial.println(backKd, 3);
}

void printLimits() {
  Serial.print(F("FT="));
  Serial.print(limitActive(FRONT_LIMIT_TOP));
  Serial.print(F(",FB="));
  Serial.print(limitActive(FRONT_LIMIT_BOTTOM));
  Serial.print(F(",BT="));
  Serial.print(limitActive(BACK_LIMIT_TOP));
  Serial.print(F(",BB="));
  Serial.println(limitActive(BACK_LIMIT_BOTTOM));
}

void executeCommand(char *command) {
  while (*command == ' ' || *command == '\t') command++;

  if (strncasecmp(command, "LP,", 3) == 0) {
    long front = 0;
    long back = 0;
    if (parseTwoLongs(command + 3, front, back)) {
      setTarget((int32_t)front, (int32_t)back);
    } else {
      Serial.println(F("ERR: use LP,frontPulse,backPulse"));
    }
    return;
  }

  if (strcasecmp(command, "S") == 0) {
    stopMotors();
    Serial.println(F("STOP"));
    return;
  }

  if (strcasecmp(command, "LZ") == 0 ||
      strcasecmp(command, "HOME") == 0) {
    beginHome();
    return;
  }

  if (strcasecmp(command, "Z") == 0) {
    zeroEncodersAndTargets();
    Serial.println(F("ENCODER ZERO"));
    return;
  }

  if (strcasecmp(command, "POSE") == 0) {
    printPose();
    return;
  }

  if (strcasecmp(command, "LIMIT") == 0) {
    printLimits();
    return;
  }

  if (strcasecmp(command, "PID") == 0) {
    printSettings();
    return;
  }

  char *end = nullptr;
  const long value = strtol(command, &end, 10);
  if (end != command && value >= 0) {
    const float gain = value / 1000.0f;
    const char first = (char)tolower((unsigned char)end[0]);
    const char second = (char)tolower((unsigned char)end[1]);

    // Individual PID: fp/fi/fd and bp/bi/bd.
    if (end[2] == '\0' && (first == 'f' || first == 'b') &&
        (second == 'p' || second == 'i' || second == 'd')) {
      float *selectedGain = nullptr;
      if (first == 'f' && second == 'p') selectedGain = &frontKp;
      else if (first == 'f' && second == 'i') selectedGain = &frontKi;
      else if (first == 'f' && second == 'd') selectedGain = &frontKd;
      else if (first == 'b' && second == 'p') selectedGain = &backKp;
      else if (first == 'b' && second == 'i') selectedGain = &backKi;
      else if (first == 'b' && second == 'd') selectedGain = &backKd;

      if (selectedGain != nullptr) {
        *selectedGain = gain;
        resetControllers();
        Serial.print(F("SET "));
        Serial.print((char)toupper((unsigned char)first));
        Serial.print((char)toupper((unsigned char)second));
        Serial.print(F("="));
        Serial.println(gain, 3);
      }
      return;
    }

    // Both PID: p/i/d.
    if (end[1] == '\0') {
      if (first == 'p') {
        frontKp = gain;
        backKp = gain;
      } else if (first == 'i') {
        frontKi = gain;
        backKi = gain;
      } else if (first == 'd') {
        frontKd = gain;
        backKd = gain;
      } else {
        Serial.println(F("ERR: use p/i/d, fp/fi/fd, bp/bi/bd"));
        return;
      }

      resetControllers();
      Serial.print(F("SET "));
      Serial.print((char)toupper((unsigned char)first));
      Serial.print(F("="));
      Serial.println(gain, 3);
      return;
    }
  }

  Serial.println(F("ERR: LP,f,b | LZ | S | POSE | PID | p/i/d"));
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char received = (char)Serial.read();

    if (received == '\r' || received == '\n') {
      if (usbCommandIndex > 0) {
        usbCommandBuffer[usbCommandIndex] = '\0';
        executeCommand(usbCommandBuffer);
        usbCommandIndex = 0;
      }
    } else if (usbCommandIndex < COMMAND_BUFFER_SIZE - 1) {
      usbCommandBuffer[usbCommandIndex++] = received;
    } else {
      usbCommandIndex = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
}
