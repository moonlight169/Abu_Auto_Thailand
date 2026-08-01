// ===========================================================================
//  ABU Auto Thailand V3 - Wheel driver  (Blackpill STM32F411CE)
//
//  4-wheel mecanum speed PID. Receives wheel RPM targets from the Master over
//  Serial1 and streams encoder/RPM feedback back at 50 Hz.
//
//  tuning + pin map : config.h / config.cpp
//  encoders         : encoder.cpp
//  H-bridge output  : motor.cpp
//  PID              : speed_pid.cpp
//  Master protocol  : master_link.cpp
//  USB test menu    : console.cpp
// ===========================================================================

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

#include "config.h"
#include "console.h"
#include "encoder.h"
#include "master_link.h"
#include "motor.h"
#include "speed_pid.h"

static uint32_t previousControlTime = 0;
static uint32_t previousStatusTime = 0;
static uint32_t previousFeedbackTime = 0;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(SERIAL1_BAUD);

  #if defined(STM32F1xx)
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
  #endif

  initializeMotorPins();
  attachEncoderInterrupts();

  stopDrive();
  resetAllControllers();
  previousControlTime = micros();

  Serial.println("Mecanum 4-wheel PID ready");
  Serial.println("Kp=1.3 Ki=1.5 Kd=0.0, CPR=691");
  Serial.println("Serial1: PA9=TX, PA10=RX, 115200 baud");
  Serial.println("Teensy command: T,FL,FR,RL,RR");
  printHelp();
}

void loop()
{
  readSimpleSerial();
  readSerial1Commands();

  if (serial1ControlActive && driveEnabled &&
      (uint32_t)(millis() - lastCommandTime) > SERIAL1_TIMEOUT_MS)
  {
    stopDrive();
    serial1ControlActive = false;
    Serial1.println("E,TIMEOUT_STOP");
    Serial.println("SERIAL1 TIMEOUT STOP");
  }

  const uint32_t currentTime = micros();
  if ((uint32_t)(currentTime - previousControlTime) >= CONTROL_PERIOD_US)
  {
    previousControlTime += CONTROL_PERIOD_US;

    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
      updateRPM(wheel);

    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
      updateSpeedPID(wheel);

    if ((uint32_t)(millis() - previousFeedbackTime) >= FEEDBACK_PERIOD_MS)
    {
      previousFeedbackTime = millis();
      sendSerial1Feedback();
    }

    if ((uint32_t)(millis() - previousStatusTime) >= STATUS_PERIOD_MS)
    {
      previousStatusTime = millis();
      printStatus();
    }
  }
}
