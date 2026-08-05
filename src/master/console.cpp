#include "console.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "arm_link.h"
#include "button_link.h"
#include "gyro.h"
#include "high_align.h"
#include "hub_link.h"
#include "kinematics.h"
#include "lift_link.h"
#include "mission_runner.h"
#include "odometry.h"
#include "position_control.h"
#include "robot_state.h"
#include "wheel_link.h"

static char usbRxBuffer[USB_RX_SIZE];
static size_t usbRxIndex = 0;

void printHelp()
{
  Serial.println("=== MASTER CONSOLE (Teensy 4.1) ===");
  Serial.println("--- mission ---");
  Serial.println("M            : start selected mission");
  Serial.println("M1 / M2 / M3 : select and start that mission");
  Serial.println("SEL 2        : select mission without starting");
  Serial.println("K0120011     : fake a keypad code (same filters as Serial3)");
  Serial.println("A            : abort mission (uppercase only; a = drive)");
  Serial.println("--- drive ---");
  Serial.println("100a / 100b  : forward / backward 100 RPM");
  Serial.println("100c / 100d  : slide left / right");
  Serial.println("100e / 100f  : rotate CCW / CW");
  Serial.println("V,0.20,0.00,0.30 : LOCAL vx,vy m/s and wz rad/s");
  Serial.println("G,0.20,0.00,0.30 : GLOBAL vx,vy m/s and wz rad/s");
  Serial.println("V,0,0,0 or G,0,0,0 : normal soft stop");
  Serial.println("P,1.00,0.50,90 : GLOBAL target X,Y m and Yaw deg");
  Serial.println("X    : cancel position control and stop");
  Serial.println("s    : immediate/emergency stop (the robot, not the Arm)");
  Serial.println("--- lift (Serial7) ---");
  Serial.println("LP,1500,1500 : lift pulse; wait until reached");
  Serial.println("LZ           : lift home; wait until reached");
  Serial.println("HIGH START   : align against higher step and zero gyro");
  Serial.println("HIGH START,3800,3900 : same, custom lift after aligning");
  Serial.println("HIGH STOP    : abort higher-step alignment");
  Serial.println("--- external Arm (Serial6) ---");
  Serial.println("AH  or HOME  : Arm home");
  Serial.println("AB 90 or B,90 : bottom axis to 90 deg (0..180)");
  Serial.println("AT 45 or T,45 : top axis to 45 deg (0..180)");
  Serial.println("AP 90 45 or POS,90,45 : both axes at once");
  Serial.println("AS  or STATUS : request Arm status");
  Serial.println("STOP / CLEAR : stop Arm / clear Arm fault");
  Serial.println("--- Hub (Serial1) ---");
  Serial.println("i            : show Hub input/output status");
  Serial.println("R1 ON/OFF    : control Hub relay 1..4 (R4 = gripper)");
  Serial.println("ARM 40       : Hub ARM servo (0..80 deg), not the Arm board");
  Serial.println("SPIN 100     : Hub SPIN servo (0..133 deg)");
  Serial.println("--- status ---");
  Serial.println("v    : link and target status");
  Serial.println("g    : show BNO085 roll, pitch and yaw");
  Serial.println("r    : reset BNO085 yaw to zero");
  Serial.println("z    : zero Teensy odometry");
  Serial.println("q    : request wheel feedback now");
  Serial.println("o    : toggle continuous robot pose print");
  Serial.println("h    : help");
  Serial.println("--- Hub buttons (pressed, not typed) ---");
  Serial.println("SW_B  (blue) : start \"MISSION 2\"");
  Serial.println("SW_RED       : start \"MISSION 1\"");
  Serial.println("SW_YELLOW    : stop, zero odometry, arm+spin+lift home,");
  Serial.println("               pulse gripper open");
  Serial.println("SW_G (green) : not used");
  Serial.println("--- keypad panel (Serial3) ---");
  Serial.println("Mode 0 -> 7 digits e.g. 0120011, Mode 1 -> 4 digits e.g. 1130");
  Serial.println("Sent 5x per press; the Master votes over 150 ms, then runs it.");
  Serial.println("Number is wheel RPM, maximum 420 RPM.");
  Serial.println("Legacy a-h commands read the LAST letter of the line.");
}

static void executeLegacyUsbCommand(const char *line)
{
  const size_t length = strlen(line);
  if (length == 0)
    return;

  const char command = (char)tolower(line[length - 1]);
  const float rpm = constrain((float)atof(line), 0.0f, MAX_RPM);

  // Any legacy movement command takes control away from Position/Velocity mode.
  if (command == 'a' || command == 'b' || command == 'c' ||
      command == 'd' || command == 'e' || command == 'f')
  {
    positionControlActive = false;
    velocityMode = VELOCITY_STOPPED;
  }

  switch (command)
  {
    case 'a':
      setTargets(rpm, rpm, rpm, rpm);
      Serial.print("FORWARD ");
      Serial.println(rpm, 1);
      sendTargetPacket();
      break;

    case 'b':
      setTargets(-rpm, -rpm, -rpm, -rpm);
      Serial.print("BACKWARD ");
      Serial.println(rpm, 1);
      sendTargetPacket();
      break;

    case 'c':
      setTargets(-rpm, rpm, rpm, -rpm);
      Serial.print("LEFT ");
      Serial.println(rpm, 1);
      sendTargetPacket();
      break;

    case 'd':
      setTargets(rpm, -rpm, -rpm, rpm);
      Serial.print("RIGHT ");
      Serial.println(rpm, 1);
      sendTargetPacket();
      break;

    case 'e':
      setTargets(-rpm, rpm, -rpm, rpm);
      Serial.print("ROTATE CCW ");
      Serial.println(rpm, 1);
      sendTargetPacket();
      break;

    case 'f':
      setTargets(rpm, -rpm, rpm, -rpm);
      Serial.print("ROTATE CW ");
      Serial.println(rpm, 1);
      sendTargetPacket();
      break;

    case 's':
      if (higherAlignActive)
        stopHigherStepAlignment("EMERGENCY STOP");
      else
      {
        stopRobot();
        Serial.println("STOP");
      }
      break;

    case 'v':
      printStatus();
      break;

    case 'z':
      resetOdometry();
      Serial.println("TEENSY ODOMETRY ZERO");
      break;

    case 'q':
      Serial2.println("Q");
      break;

    case 'r':
      resetGyroYaw();
      break;

    case 'g':
      printSensorStatus();
      break;

    case 'o':
      posePrintEnabled = !posePrintEnabled;
      Serial.print("POSE PRINT ");
      Serial.println(posePrintEnabled ? "ON" : "OFF");
      if (posePrintEnabled)
        printRobotPose();
      break;

    case 'h':
      printHelp();
      break;

    default:
      Serial.print("ERR unknown command: [");
      Serial.print(line);
      Serial.println("] use H");
      break;
  }
}

void processUsbLine(char *line)
{
  while (*line == ' ' || *line == '\t')
    line++;

  // Remove trailing spaces/tabs. Some Serial Monitor settings append them,
  // which previously made "HIGH START " fall through as an unknown command.
  size_t lineLength = strlen(line);
  while (lineLength > 0 &&
         (line[lineLength - 1] == ' ' || line[lineLength - 1] == '\t'))
  {
    line[--lineLength] = '\0';
  }

  if (lineLength == 0)
    return;

  long liftFrontPulse = 0;
  long liftBackPulse = 0;
  int hubNumber = 0;
  int hubValue = 0;
  float armBottomDeg = 0.0f;
  float armTopDeg = 0.0f;
  char hubState[8] = {};

  // USB Serial Monitor accepts HOME, CMD,HOME, or CMD,<seq>,HOME.
  // The Master always allocates its own sequence before forwarding to the Arm.
  if (strncasecmp(line, "CMD,", 4) == 0)
  {
    line += 4;
    char *comma = strchr(line, ',');
    if (comma != nullptr)
    {
      bool numericSequence = comma != line;
      for (char *p = line; p < comma; ++p)
      {
        if (*p < '0' || *p > '9')
        {
          numericSequence = false;
          break;
        }
      }
      if (numericSequence)
        line = comma + 1;
    }
  }

  long highAfterFrontPulse =
      HIGH_DEFAULT_AFTER_LIFT_FRONT_PULSE;
  long highAfterBackPulse =
      HIGH_DEFAULT_AFTER_LIFT_BACK_PULSE;

  if (sscanf(line, "HIGH START,%ld,%ld",
             &highAfterFrontPulse,
             &highAfterBackPulse) == 2 ||
      sscanf(line, "high start,%ld,%ld",
             &highAfterFrontPulse,
             &highAfterBackPulse) == 2)
  {
    startHigherStepAlignment(
        (int32_t)highAfterFrontPulse,
        (int32_t)highAfterBackPulse);
    return;
  }

  if (strcasecmp(line, "HIGH START") == 0)
  {
    startHigherStepAlignment(
        HIGH_DEFAULT_AFTER_LIFT_FRONT_PULSE,
        HIGH_DEFAULT_AFTER_LIFT_BACK_PULSE);
    return;
  }

  if (strcasecmp(line, "HIGH STOP") == 0)
  {
    stopHigherStepAlignment("USB ABORT");
    return;
  }

  if (strcasecmp(line, "HOME") == 0)
  {
    startArmHome();
    return;
  }

  if (sscanf(line, "POS,%f,%f", &armBottomDeg, &armTopDeg) == 2 ||
      sscanf(line, "pos,%f,%f", &armBottomDeg, &armTopDeg) == 2)
  {
    startArmPosition(armBottomDeg, armTopDeg);
    return;
  }

  if (sscanf(line, "B,%f", &armBottomDeg) == 1 ||
      sscanf(line, "b,%f", &armBottomDeg) == 1)
  {
    startArmBottom(armBottomDeg);
    return;
  }

  if (sscanf(line, "T,%f", &armTopDeg) == 1 ||
      sscanf(line, "t,%f", &armTopDeg) == 1)
  {
    startArmTop(armTopDeg);
    return;
  }

  if (strcasecmp(line, "STATUS") == 0)
  {
    sendArmAuxCommand("STATUS");
    return;
  }

  if (strcasecmp(line, "STOP") == 0)
  {
    sendArmAuxCommand("STOP");
    armTaskStatus = ARM_TASK_IDLE;
    armExpectedDone[0] = '\0';
    armTxPacket[0] = '\0';
    return;
  }

  if (strcasecmp(line, "CLEAR") == 0)
  {
    sendArmAuxCommand("CLEAR");
    armTaskStatus = ARM_TASK_IDLE;
    armExpectedDone[0] = '\0';
    armTxPacket[0] = '\0';
    return;
  }

  if (strcasecmp(line, "AH") == 0 ||
      strcasecmp(line, "ARM HOME") == 0)
  {
    startArmHome();
    return;
  }

  if (sscanf(line, "AP %f %f", &armBottomDeg, &armTopDeg) == 2 ||
      sscanf(line, "ap %f %f", &armBottomDeg, &armTopDeg) == 2)
  {
    startArmPosition(armBottomDeg, armTopDeg);
    return;
  }

  if (sscanf(line, "AB %f", &armBottomDeg) == 1 ||
      sscanf(line, "ab %f", &armBottomDeg) == 1)
  {
    startArmBottom(armBottomDeg);
    return;
  }

  if (sscanf(line, "AT %f", &armTopDeg) == 1 ||
      sscanf(line, "at %f", &armTopDeg) == 1)
  {
    startArmTop(armTopDeg);
    return;
  }

  if (strcasecmp(line, "AS") == 0 ||
      strcasecmp(line, "ARM STATUS") == 0)
  {
    sendArmAuxCommand("STATUS");
    return;
  }

  // "K0120011" pretends to be a keypad frame. It goes through submitMissionCode()
  // exactly like Serial3 does -- same filters, same vote window -- so the whole
  // path can be tested from the Serial Monitor with no keypad board attached.
  // Type it 5 times quickly to rehearse a real burst.
  if ((line[0] == 'K' || line[0] == 'k') && line[1] != '\0')
  {
    submitMissionCode(line + 1, "CONSOLE");
    return;
  }

  if (strcmp(line, "M") == 0 || strcmp(line, "m") == 0)
  {
    startMission();
    return;
  }

  int missionNumber = 0;
  if ((sscanf(line, "M%d", &missionNumber) == 1 ||
       sscanf(line, "m%d", &missionNumber) == 1) &&
      missionNumber > 0)
  {
    startMission((uint8_t)missionNumber);
    return;
  }

  if ((sscanf(line, "SEL %d", &missionNumber) == 1 ||
       sscanf(line, "sel %d", &missionNumber) == 1) &&
      missionNumber > 0)
  {
    selectMission((uint8_t)missionNumber);
    return;
  }

  if (strcmp(line, "A") == 0)
  {
    stopMission("USB ABORT");
    return;
  }

  if (strcmp(line, "i") == 0 || strcmp(line, "I") == 0)
  {
    printHubStatus();
    return;
  }

  if ((sscanf(line, "R%d %7s", &hubNumber, hubState) == 2 ||
       sscanf(line, "r%d %7s", &hubNumber, hubState) == 2) &&
      hubNumber >= 1 && hubNumber <= 4)
  {
    const bool on = strcasecmp(hubState, "ON") == 0 ||
                    strcmp(hubState, "1") == 0;
    setHubRelay((uint8_t)hubNumber, on);
    return;
  }

  if (sscanf(line, "ARM %d", &hubValue) == 1 ||
      sscanf(line, "arm %d", &hubValue) == 1)
  {
    setHubArm((uint8_t)constrain(hubValue, 0, (int)HUB_ARM_MAX_DEG));
    return;
  }

  if (sscanf(line, "SPIN %d", &hubValue) == 1 ||
      sscanf(line, "spin %d", &hubValue) == 1)
  {
    setHubSpin((uint8_t)constrain(hubValue, 0, (int)HUB_SPIN_MAX_DEG));
    return;
  }

  const bool liftPulseCommand =
      sscanf(line, "LP,%ld,%ld",
             &liftFrontPulse, &liftBackPulse) == 2 ||
      sscanf(line, "lp,%ld,%ld",
             &liftFrontPulse, &liftBackPulse) == 2;

  if (liftPulseCommand)
  {
    startLift((int32_t)liftFrontPulse,
              (int32_t)liftBackPulse);
    Serial.println("LIFT_COMMAND_STARTED");
    return;
  }

  if (strcmp(line, "LZ") == 0 ||
      strcmp(line, "lz") == 0 ||
      strcmp(line, "Lz") == 0 ||
      strcmp(line, "lZ") == 0)
  {
    startLiftHome();
    Serial.println("LIFT_HOME_STARTED");
    return;
  }

  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;

  float targetX = 0.0f;
  float targetY = 0.0f;
  float targetYawDeg = 0.0f;
  const bool positionCommand =
      sscanf(line, "P,%f,%f,%f",
             &targetX, &targetY, &targetYawDeg) == 3 ||
      sscanf(line, "p,%f,%f,%f",
             &targetX, &targetY, &targetYawDeg) == 3;

  if (positionCommand)
  {
    startMoveTo(targetX, targetY, targetYawDeg);
    return;
  }

  if (strcmp(line, "X") == 0 || strcmp(line, "x") == 0)
  {
    cancelPositionControl();
    return;
  }

  const bool localCommand =
      sscanf(line, "V,%f,%f,%f", &vx, &vy, &wz) == 3 ||
      sscanf(line, "v,%f,%f,%f", &vx, &vy, &wz) == 3;

  if (localCommand)
  {
    positionControlActive = false;
    commandLocalVelocity(vx, vy, wz);
    commandActive = true;
    updateVelocityControl();
    Serial.println(
        fabsf(vx) < RAMP_ZERO_EPSILON &&
        fabsf(vy) < RAMP_ZERO_EPSILON &&
        fabsf(wz) < RAMP_ZERO_EPSILON
            ? "SOFT STOP START"
            : "LOCAL VELOCITY RAMP START");
    return;
  }

  const bool globalCommand =
      sscanf(line, "G,%f,%f,%f", &vx, &vy, &wz) == 3 ||
      sscanf(line, "g,%f,%f,%f", &vx, &vy, &wz) == 3;

  if (globalCommand)
  {
    positionControlActive = false;
    commandGlobalVelocity(vx, vy, wz);
    commandActive = true;
    updateVelocityControl();
    Serial.println(
        fabsf(vx) < RAMP_ZERO_EPSILON &&
        fabsf(vy) < RAMP_ZERO_EPSILON &&
        fabsf(wz) < RAMP_ZERO_EPSILON
            ? "SOFT STOP START"
            : "GLOBAL VELOCITY RAMP START");
    return;
  }

  executeLegacyUsbCommand(line);
}

void readUsbSerial()
{
  while (Serial.available() > 0)
  {
    const char received = (char)Serial.read();

    if (received == '\r' || received == '\n')
    {
      if (usbRxIndex > 0)
      {
        usbRxBuffer[usbRxIndex] = '\0';
        processUsbLine(usbRxBuffer);
        usbRxIndex = 0;
      }
      continue;
    }

    if (received >= 32 && received <= 126 &&
        usbRxIndex < USB_RX_SIZE - 1)
    {
      usbRxBuffer[usbRxIndex++] = received;
    }
    else if (usbRxIndex >= USB_RX_SIZE - 1)
    {
      usbRxIndex = 0;
      Serial.println("USB COMMAND TOO LONG");
    }
  }
}
