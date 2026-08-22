#include "console.h"

#include <string.h>

#include "io_board.h"
#include "lidar.h"
#include "lidar_monitor.h"
#include "master_link.h"
#include "tfmini.h"

void printHelp() {
  Serial.println();
  Serial.println("========== SENSOR HUB TEST ==========");
  Serial.println("R1 ON / R1 OFF        Relay 1");
  Serial.println("R2 ON/OFF ... R4      Relay 2-4 (R4 = gripper, ON = open)");
  Serial.println("ALL ON / ALL OFF      All relays");
  Serial.println("STOP                  All relays OFF, not a robot stop");
  Serial.println("ARM 0..80             Arm servo angle");
  Serial.println("SPIN 0..133           Spin servo angle");
  Serial.println("INPUT                 Show inputs");
  Serial.println("OUTPUT                Show outputs");
  Serial.println("STATUS                Show all I/O");
  Serial.println("LIDAR ON              Realtime summary + box bar (= M2)");
  Serial.println("LIDAR OFF             Stop USB LiDAR reporting (= M0)");
  Serial.println("LIDAR STATUS          Print one LiDAR result");
  Serial.println("LIDAR RESET  or R     Reset detection filter");
  Serial.println("MODE BOX     or BOX   Find box distance/center/angle");
  Serial.println("MODE WALL    or WALL  Find front plane distance/angle");
  Serial.println("MODE?        or MODE  Show active LiDAR mode");
  Serial.println("M0 / M1 / M2 / M3     Off / summary / bar / CSV points");
  Serial.println("D450                  Score target distance mm (100-2000)");
  Serial.println("A0.45                 Filter alpha (0.01-1.00)");
  Serial.println("T250                  Print interval ms (50-5000)");
  Serial.println("HELP         or M?    Show commands");
  Serial.println("--- MASTER Serial1 commands, typeable here too ---");
  Serial.println("GET LIDAR             One LiDAR result");
  Serial.println("GET IO                Inputs as bit mask");
  Serial.println("GET SERVO             Current servo angles");
  Serial.println("GET TFMINI / GET TF   TFMini 1/2/3/4 distance in cm");
  Serial.println("GET STATUS            Complete hub packet");
  Serial.println("STREAM ON/OFF         Complete stream to Master");
  Serial.println("Case does not matter. A trailing space breaks the match.");
  Serial.println("=====================================");
  Serial.println();
}

// fromMaster=false: command came from USB Serial Monitor.
// fromMaster=true : command came from Serial1 and receives an ACK.
void processCommand(char *command, bool fromMaster) {
  for (uint8_t i = 0; command[i] != '\0'; i++) {
    if (command[i] >= 'a' && command[i] <= 'z') {
      command[i] -= 32;
    }
  }
  while (*command == ' ') {
    command++;
  }

  if (strcmp(command, "R1 ON") == 0) {
    setRelay(1, true);
  } else if (strcmp(command, "R1 OFF") == 0) {
    setRelay(1, false);
  } else if (strcmp(command, "R2 ON") == 0) {
    setRelay(2, true);
  } else if (strcmp(command, "R2 OFF") == 0) {
    setRelay(2, false);
  } else if (strcmp(command, "R3 ON") == 0) {
    setRelay(3, true);
  } else if (strcmp(command, "R3 OFF") == 0) {
    setRelay(3, false);
  } else if (strcmp(command, "R4 ON") == 0) {
    setRelay(4, true);
  } else if (strcmp(command, "R4 OFF") == 0) {
    setRelay(4, false);
  } else if (strcmp(command, "ALL ON") == 0) {
    setAllRelays(true);
  } else if (strcmp(command, "ALL OFF") == 0 ||
             strcmp(command, "STOP") == 0) {
    setAllRelays(false);
  } else if (strncmp(command, "ARM ", 4) == 0) {
    setArmServo(atoi(command + 4));
    Serial.print("ARM SERVO=");
    Serial.println(armAngleDeg);
  } else if (strncmp(command, "SPIN ", 5) == 0) {
    setSpinServo(atoi(command + 5));
    Serial.print("SPIN SERVO=");
    Serial.println(spinAngleDeg);
  } else if (strcmp(command, "INPUT") == 0) {
    printAllInputs();
  } else if (strcmp(command, "OUTPUT") == 0) {
    printAllOutputs();
  } else if (strcmp(command, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(command, "LIDAR ON") == 0) {
    MONITOR_MODE = 2;
    Serial.println("LIDAR MONITOR=ON");
  } else if (strcmp(command, "LIDAR OFF") == 0) {
    MONITOR_MODE = 0;
    Serial.println("LIDAR MONITOR=OFF");
  } else if (strcmp(command, "LIDAR STATUS") == 0) {
    printMonitorSummary();
  } else if (strcmp(command, "GET LIDAR") == 0) {
    sendMasterLidar();
  } else if (strcmp(command, "GET IO") == 0) {
    sendMasterIo();
  } else if (strcmp(command, "GET SERVO") == 0) {
    sendMasterServo();
  } else if (strcmp(command, "GET TFMINI") == 0 ||
             strcmp(command, "GET TF") == 0) {
    sendMasterTfmini();
  } else if (strcmp(command, "GET STATUS") == 0) {
    sendMasterStatus();
  } else if (strcmp(command, "STREAM ON") == 0) {
    masterStreamEnabled = true;
  } else if (strcmp(command, "STREAM OFF") == 0) {
    masterStreamEnabled = false;
  } else if (strcmp(command, "LIDAR RESET") == 0 ||
             strcmp(command, "R") == 0) {
    resetDetectionFilter();
    Serial.println("LIDAR FILTER=RESET");
  } else if (strcmp(command, "MODE BOX") == 0 ||
             strcmp(command, "BOX") == 0) {
    setDetectionMode(MODE_BOX);
  } else if (strcmp(command, "MODE WALL") == 0 ||
             strcmp(command, "WALL") == 0) {
    setDetectionMode(MODE_WALL);
  } else if (strcmp(command, "MODE?") == 0 ||
             strcmp(command, "MODE") == 0) {
    Serial.print("LIDAR MODE=");
    Serial.println(detectionMode == MODE_BOX ? "BOX" : "WALL");
  } else if (strcmp(command, "HELP") == 0 ||
             strcmp(command, "M?") == 0) {
    printHelp();
  } else if (strcmp(command, "M0") == 0) {
    MONITOR_MODE = 0;
    Serial.println("MONITOR=OFF");
  } else if (strcmp(command, "M1") == 0) {
    MONITOR_MODE = 1;
    Serial.println("MONITOR=SUMMARY");
  } else if (strcmp(command, "M2") == 0) {
    MONITOR_MODE = 2;
    Serial.println("MONITOR=BOX_BAR");
  } else if (strcmp(command, "M3") == 0) {
    MONITOR_MODE = 3;
    Serial.println("MONITOR=CSV_POINTS");
  } else if (command[0] == 'D') {
    const float value = atof(command + 1);
    if (value >= 100.0f && value <= 2000.0f) {
      TARGET_DIST_MM = value;
      filterReady = false;
      Serial.print("TARGET_DIST_MM=");
      Serial.println(TARGET_DIST_MM, 0);
    } else {
      Serial.println("ERROR,D range 100-2000");
    }
  } else if (command[0] == 'A') {
    const float value = atof(command + 1);
    if (value >= 0.01f && value <= 1.0f) {
      FILTER_ALPHA = value;
      Serial.print("FILTER_ALPHA=");
      Serial.println(FILTER_ALPHA, 2);
    } else {
      Serial.println("ERROR,A range 0.01-1.00");
    }
  } else if (command[0] == 'T') {
    const int value = atoi(command + 1);
    if (value >= 50 && value <= 5000) {
      MONITOR_INTERVAL_MS = value;
      Serial.print("MONITOR_INTERVAL_MS=");
      Serial.println(MONITOR_INTERVAL_MS);
    } else {
      Serial.println("ERROR,T range 50-5000");
    }
  } else if (strlen(command) > 0) {
    Serial.print("UNKNOWN COMMAND: ");
    Serial.println(command);
    if (fromMaster) {
      master.print("ERR,UNKNOWN,");
      master.println(command);
    }
    return;
  }

  if (fromMaster && strlen(command) > 0 &&
      strncmp(command, "GET ", 4) != 0) {
    master.print("ACK,");
    master.println(command);
  }
}

static void handleCommandPort(Stream &port, bool fromMaster,
                              char *commandBuffer, uint8_t &commandIndex,
                              size_t bufferSize) {
  while (port.available() > 0) {
    const char received = port.read();
    if (received == '\r') {
      continue;
    }
    if (received == '\n') {
      commandBuffer[commandIndex] = '\0';
      processCommand(commandBuffer, fromMaster);
      commandIndex = 0;
      continue;
    }
    if (commandIndex < bufferSize - 1) {
      commandBuffer[commandIndex++] = received;
    } else {
      commandIndex = 0;
      if (fromMaster) {
        master.println("ERR,TOO_LONG");
      }
    }
  }
}

void handleCommands() {
  static char usbBuffer[48];
  static uint8_t usbIndex = 0;
  static char masterBuffer[48];
  static uint8_t masterIndex = 0;

  handleCommandPort(Serial, false, usbBuffer, usbIndex, sizeof(usbBuffer));
  handleCommandPort(master, true, masterBuffer, masterIndex,
                    sizeof(masterBuffer));
}