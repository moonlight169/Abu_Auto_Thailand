// ===========================================================================
//  ABU Auto Thailand V3 - Sensor hub  (Teensy 4.1)
//
//  Owns the RPLiDAR (box / wall detection), four TFMini-S range finders, the
//  relay board, the two servos and all the field switches. Streams one
//  complete "HUB,..." packet to the Master at 20 Hz.
//
//  tuning + pin map : config.h / config.cpp
//  relays/IO/servos : io_board.cpp
//  TFMini-S         : tfmini.cpp
//  LiDAR processing : lidar.cpp
//  LiDAR USB debug  : lidar_monitor.cpp
//  Master protocol  : master_link.cpp
//  command parser   : console.cpp
// ===========================================================================

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "console.h"
#include "io_board.h"
#include "lidar.h"
#include "lidar_monitor.h"
#include "master_link.h"
#include "tfmini.h"

void setup() {
  Serial.begin(USB_BAUD);
  master.begin(MASTER_BAUD);
  while (!Serial && millis() < 3000) {
  }

  initializeIoBoard();

  clearResult();
  beginTfmini();
  beginLidar();

  Serial.println();
  Serial.println("=== ABU SENSOR HUB + LIDAR TEST ===");
  Serial.println("INPUT ACTIVE LOW / RELAY LOW = ON");
  Serial.println("Master Serial1 @ 460800 baud (RX1=0, TX1=1)");
  Serial.println("LiDAR Serial8 @ 460800 baud (RX8=34, TX8=35)");
  Serial.println("TFMini-S Serial2 / Serial6 / Serial7 / Serial3 @ 115200 baud");
  Serial.println("TF4 Serial3 pins: RX3=15, TX3=14");
  Serial.println("ARM range 0-80 / SPIN range 0-133");
  Serial.println("LiDAR USB monitor starts OFF");
  printHelp();
}

void loop() {
  handleCommands();
  updateInputs();
  updateTfmini();
  // printTfminiMonitor();
  readLidar();
  updateMonitor();
  updateMasterStream();
}
