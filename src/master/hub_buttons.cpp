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

// --- ตัวแปรสำหรับจัดการการกดปุ่มพร้อมกัน (Combo Window) ---
static bool pendingBlue = false;
static bool pendingRed = false;
static uint32_t pendingBlueMs = 0;
static uint32_t pendingRedMs = 0;
// ระยะเวลารอให้กดปุ่มที่ 2 (100 มิลลิวินาที คือระดับที่มนุษย์ไม่รู้สึกว่าช้า แต่โค้ดทำงานได้ชัวร์)
static constexpr uint32_t COMBO_WINDOW_MS = 100; 

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
  setHubRelay(2, 0);
  setHubRelay(3, 0);

  setHubArm(10);
  setHubSpin(0);
  startLiftHome();

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

// Addressed by name, never by position.
static constexpr const char *BLUE_BUTTON_MISSION = "MISSION 2";  // mission2
static constexpr const char *RED_BUTTON_MISSION = "MISSION 1";   // mission1

static void handleHubButtonPressed(uint8_t bit)
{
  // 1. นำ Combo logic ออกจากตรงนี้ แล้วให้ทำงานเฉพาะปุ่มเดี่ยว
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
        
        // 2. เมื่อปุ่มถูกกด แทนที่จะรัน Mission ทันที ให้ตั้งธงพักไว้ (Pending) ก่อน
        if (HUB_BUTTON_BITS[i] == HUB_SW_BLUE)
        {
          pendingBlue = true;
          pendingBlueMs = now;
        }
        else if (HUB_BUTTON_BITS[i] == HUB_SW_RED)
        {
          pendingRed = true;
          pendingRedMs = now;
        }
        else
        {
          // ปุ่มเหลือง หรือปุ่มอื่นๆ ให้ทำงานทันทีไม่ต้องรอ Combo
          handleHubButtonPressed(HUB_BUTTON_BITS[i]);
        }
      }
      else
      {
        hubButtonStableMask &= (uint16_t)~mask;
      }
    }
  }

  // 3. ตรวจสอบสถานะ Combo นอก Loop
  if (pendingBlue && pendingRed)
  {
    // หากมีการกดทั้งสองปุ่มค้างไว้ในระบบ (กดห่างกันไม่เกิน 100ms)
    Serial.println("SW_BLUE + SW_RED PRESSED - ARM CLEAR AND HOME");
    
    // บังคับหยุด Mission กรณีหลุดรอดมา และหยุด Robot ด้วยเพื่อความปลอดภัย
    if (missionRunning) stopMission("COMBO OVERRIDE");
    
    startArmClearAndHome();
    
    // ล้างสถานะเพื่อไม่ให้ทำงานซ้ำ
    pendingBlue = false;
    pendingRed = false;
  }
  else
  {
    // 4. ถ้ากดแค่ปุ่มเดียว แล้วเวลาผ่านไปเกิน 100ms (แน่ใจแล้วว่าไม่ได้กด 2 ปุ่ม) จึงค่อยรัน Mission
    if (pendingBlue && (uint32_t)(now - pendingBlueMs) >= COMBO_WINDOW_MS)
    {
      handleHubButtonPressed(HUB_SW_BLUE);
      pendingBlue = false;
    }
    
    if (pendingRed && (uint32_t)(now - pendingRedMs) >= COMBO_WINDOW_MS)
    {
      handleHubButtonPressed(HUB_SW_RED);
      pendingRed = false;
    }
  }
}