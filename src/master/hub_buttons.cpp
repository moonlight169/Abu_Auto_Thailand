#include "hub_buttons.h"

#include "arm_link.h"
#include "gyro.h"
#include "hub_link.h"
#include "kinematics.h"
#include "lift_link.h"
#include "mission_runner.h"
#include "odometry.h"
#include "types.h"

static constexpr uint8_t HUB_BUTTON_BITS[] =
{
  HUB_SW_BLUE,
  HUB_SW_RED,
  HUB_SW_YELLOW
};

static uint16_t hubButtonRawMask = 0;
static uint16_t hubButtonStableMask = 0;
static uint32_t hubButtonChangedMs[ARRAY_COUNT(HUB_BUTTON_BITS)] = {};
static bool hubButtonsInitialized = false;

// SW_Y opens the gripper for YELLOW_RELAY4_PULSE_MS, then closes it again.
static bool yellowRelay4PulseActive = false;
static uint32_t yellowRelay4PulseStartMs = 0;

void toggleGripper()
{
  if (missionRunning)
  {
    Serial.println("SW_A IGNORED - MISSION IS RUNNING");
    return;
  }

  // Relay 4 ON = gripper open, Relay 4 OFF = gripper closed.
  // Read the actual relay state reported by the HUB before toggling it.
  const bool gripperIsOpen =
      (hub.relayMask & (1u << (GRIPPER_RELAY_NUMBER - 1))) != 0;
  const bool openGripper = !gripperIsOpen;

  setHubRelay(GRIPPER_RELAY_NUMBER, openGripper);
  Serial.println(openGripper
                     ? "SW_A PRESSED - GRIPPER OPEN"
                     : "SW_A PRESSED - GRIPPER CLOSE");
}

static void onSwitchYellow()
{
  if (missionRunning)
    stopMission("SW_Y RESET");
  else
    stopRobot();

  resetOdometry();
  resetGyroYaw();

  setHubRelay(1, 0);
  setHubRelay(2, 1);
  setHubRelay(3, 1);

  setHubArm(5);
  setHubSpin(0);
  startLiftHome();

  startArmClearAndHome();

  setHubRelay(4, 1);
  yellowRelay4PulseActive = true;
  yellowRelay4PulseStartMs = millis();
}

static void updateYellowRelay4Pulse()
{
  if (!yellowRelay4PulseActive)
    return;

  if ((uint32_t)(millis() - yellowRelay4PulseStartMs) >= YELLOW_RELAY4_PULSE_MS)
  {
    setHubRelay(4, 0);
    yellowRelay4PulseActive = false;
  }
}

// Addressed by name, never by position. The registry holds 117 entries now
// and grows every time a keypad pattern is filled in; an index would silently
// point somewhere else the moment anything is inserted or reordered. A typo
// here shows up at run time as UNKNOWN MISSION CODE instead of at compile
// time, which is the one thing given up in exchange.
static constexpr const char *BLUE_BUTTON_MISSION = "MISSION 2";  // mission2
static constexpr const char *RED_BUTTON_MISSION = "MISSION 1";   // mission1

static void handleHubButtonPressed(uint8_t bit)
{
  switch (bit)
  {
    case HUB_SW_BLUE:
      Serial.println("SW_BLUE PRESSED - START MISSION 2");
      startMissionByCode(BLUE_BUTTON_MISSION);
      break;

    case HUB_SW_RED:
      Serial.println("SW_RED PRESSED - START MISSION 1");
      startMissionByCode(RED_BUTTON_MISSION);
      break;

    case HUB_SW_YELLOW:
      onSwitchYellow();
      break;
  }
}

void updateHubButtons()
{
  updateYellowRelay4Pulse();

  if (!hub.online)
  {
    hubButtonsInitialized = false;
    return;
  }

  const uint16_t buttonMask =
      (1u << HUB_SW_BLUE) |
      (1u << HUB_SW_RED) |
      (1u << HUB_SW_YELLOW);
  const uint16_t rawMask = hub.inputMask & buttonMask;
  const uint32_t now = millis();

  if (!hubButtonsInitialized)
  {
    hubButtonRawMask = rawMask;
    hubButtonStableMask = rawMask;
    for (size_t i = 0; i < ARRAY_COUNT(HUB_BUTTON_BITS); i++)
      hubButtonChangedMs[i] = now;
    hubButtonsInitialized = true;
    return;
  }

  for (size_t i = 0; i < ARRAY_COUNT(HUB_BUTTON_BITS); i++)
  {
    const uint16_t mask = (uint16_t)(1u << HUB_BUTTON_BITS[i]);
    const bool rawPressed = (rawMask & mask) != 0;
    const bool previousRawPressed = (hubButtonRawMask & mask) != 0;
    const bool stablePressed = (hubButtonStableMask & mask) != 0;

    if (rawPressed != previousRawPressed)
    {
      if (rawPressed)
        hubButtonRawMask |= mask;
      else
        hubButtonRawMask &= (uint16_t)~mask;
      hubButtonChangedMs[i] = now;
      continue;
    }

    if (rawPressed != stablePressed &&
        (uint32_t)(now - hubButtonChangedMs[i]) >= HUB_BUTTON_DEBOUNCE_MS)
    {
      if (rawPressed)
      {
        hubButtonStableMask |= mask;
        handleHubButtonPressed(HUB_BUTTON_BITS[i]);
      }
      else
      {
        hubButtonStableMask &= (uint16_t)~mask;
      }
    }
  }
}
