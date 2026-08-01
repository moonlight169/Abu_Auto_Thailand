// ===========================================================================
//  ABU Auto Thailand V3 - Arm slave  (Blackpill STM32F411CE)
//
//  Two independent encoder-PID axes (Bottom / Top) with limit-switch homing,
//  driven either from the USB test console or from the Teensy Master over
//  Serial1 using the CSV command protocol.
//
//  tuning + pin map : config.h
//  encoders         : encoder.cpp
//  H-bridge + safety: motor.cpp
//  debounced limits : arm_limits.cpp
//  state machine    : arm_control.cpp
//  Master protocol  : master_link.cpp
//  USB test menu    : console.cpp
// ===========================================================================

#include <Arduino.h>

#include "arm_control.h"
#include "arm_limits.h"
#include "config.h"
#include "console.h"
#include "encoder.h"
#include "master_link.h"
#include "motor.h"

void setup() {
  Serial.begin(115200);
  Serial1.setRx(PA10);
  Serial1.setTx(PA9);
  Serial1.begin(MASTER_BAUD);

  initializeMotorPins();
  attachEncoderInterrupts();

  bottomMotor(0);
  topMotor(0);
  initializeConsole();

  delay(300);
  updateDebouncedLimits();
  delay(LIMIT_DEBOUNCE_MS);
  updateDebouncedLimits();
  printHelp();
  printStatus();
  Serial1.print(F("READY,ARM_SLAVE,"));
  Serial1.println(CSV_PROTOCOL_VERSION);
}

void loop() {
  readSerialCommands();
  readMasterCommands();
  updateDebouncedLimits();
  updateStateMachine();
  serviceMasterJob();

  static uint32_t lastStatusMs = 0;
  static uint32_t lastLimitStreamMs = 0;

  if (limitStreamEnabled &&
      millis() - lastLimitStreamMs >= LIMIT_STREAM_PERIOD_MS) {
    lastLimitStreamMs = millis();
    printLimits();
  }

  if (!limitStreamEnabled && millis() - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = millis();
    printStatus();
  }
}
