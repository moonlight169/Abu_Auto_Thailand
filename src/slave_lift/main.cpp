// ===========================================================================
//  ABU Auto Thailand V3 - Lift slave  (Blackpill STM32F411CE)
//
//  Two independent encoder-PID lift columns (Front / Back) with limit-switch
//  homing and a soft ramp on the way down, driven either from the USB test
//  console or from the Teensy Master over Serial1.
//
//  tuning + pin map : config.h
//  encoders         : encoder.cpp
//  H-bridge         : motor.cpp
//  PID + down ramp  : position_pid.cpp
//  homing + control : lift_control.cpp
//  Master protocol  : master_link.cpp
//  USB test menu    : console.cpp
// ===========================================================================

#include <Arduino.h>

#include "config.h"
#include "console.h"
#include "encoder.h"
#include "lift_control.h"
#include "master_link.h"
#include "motor.h"

static uint32_t lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  Serial1.setRx(MASTER_LINK_RX_PIN);
  Serial1.setTx(MASTER_LINK_TX_PIN);
  Serial1.begin(MASTER_BAUD);

  initializeMotorPins();
  attachEncoderInterrupts();

  stopMotors();

  printHelp();
}

void loop() {
  readSerialCommands();
  readMasterCommands();
  updateControl();
  sendLiftPosition();

  const uint32_t now = millis();
  if ((uint32_t)(now - lastPrintTime) >= PRINT_PERIOD_MS) {
    lastPrintTime = now;
    printPose();
  }
}
