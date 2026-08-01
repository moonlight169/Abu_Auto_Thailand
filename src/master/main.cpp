// ===========================================================================
//  ABU Auto Thailand V3 - Master brain  (Teensy 4.1)
//
//  Runs the mission sequencer, mecanum odometry and the world-frame position
//  controller, and talks to every other board:
//    Serial1 -> sensor hub      Serial2 -> wheel driver
//    Serial6 -> arm slave       Serial7 -> lift controller
//
//  ---------------------------------------------------------------------
//   file                what to edit it for
//  ---------------------------------------------------------------------
//   config.h            EVERY tuning number: gains, speeds, timeouts,
//                       tolerances, TF thresholds, baud rates
//   mission_list.cpp    the actual mission sequences  <-- start here
//   mission_steps.h     the *_STEP() vocabulary the missions are built from
//   mission_runner.cpp  what each step type actually does
//   console.cpp         USB Serial Monitor commands
//   types.h             shared enums and structs
//  ---------------------------------------------------------------------
//   hub_link / wheel_link / lift_link / arm_link   one serial protocol each
//   kinematics / odometry / position_control       motion maths
//   box_parking / high_align                       LiDAR routines
//   gyro / hub_buttons / tasks / robot_state       supporting modules
// ===========================================================================

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "arm_link.h"
#include "box_parking.h"
#include "box_result.h"
#include "config.h"
#include "console.h"
#include "gyro.h"
#include "high_align.h"
#include "hub_buttons.h"
#include "hub_link.h"
#include "kinematics.h"
#include "lift_link.h"
#include "mission_runner.h"
#include "odometry.h"
#include "position_control.h"
#include "robot_state.h"
#include "tasks.h"
#include "wheel_link.h"

void setup()
{
  Serial.begin(USB_BAUD);
  beginHubSerial();
  Serial2.begin(WHEEL_BAUD);
  Serial6.begin(ARM_BAUD);
  Serial7.begin(LIFT_BAUD);

  delay(500);

  clearBoxResult();
  Serial.println("LIDAR SOURCE = HUB VIA SERIAL1");

  beginGyro();
  stopRobot();
  Serial2.println("Q");
  Serial1.println("STREAM ON");
  sendArmAuxCommand("STATUS");

  printHelp();

  resetBeforeMission();
}

void loop()
{
  readUsbSerial();
  readHubSerial();
  readSlaveSerial();
  readArmSerial();
  updateArmCommunication();
  updateArmClearAndHome();
  readLiftSerial();
  readGyro();
  updateHubButtons();

  const uint32_t now = millis();
  if (hub.online &&
      (uint32_t)(now - hub.lastPacketMs) > HUB_TIMEOUT_MS)
  {
    hub.online = false;
    hub.inputMask = 0;
    hub.relayMask = 0;
    hub.tfValidMask = 0;
    Serial.println("WARNING: HUB TIMEOUT");
  }
  if (higherAlignActive)
  {
    updateHigherStepAlignment();
  }
  else if (boxParkingActive)
  {
    updateBoxParking();
  }
  else if (positionControlActive)
  {
    updatePositionControl();
  }
  else if (commandActive &&
           (uint32_t)(now - lastCommandSendMs) >= COMMAND_PERIOD_MS)
  {
    updateVelocityControl();
  }

  updateRobotTasks();
  updateMission();
  updatePosePrint();
  updateLidarPrint();

  if (feedbackOnline &&
      (uint32_t)(now - lastFeedbackMs) > FEEDBACK_TIMEOUT_MS)
  {
    feedbackOnline = false;
    Serial.println("WARNING: STM32 FEEDBACK TIMEOUT");
  }

  if (gyroOnline &&
      (uint32_t)(now - lastGyroMs) > GYRO_TIMEOUT_MS)
  {
    gyroOnline = false;
    Serial.println("WARNING: BNO085 GYRO TIMEOUT");
  }
}
