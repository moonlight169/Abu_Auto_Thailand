#pragma once

#include <Arduino.h>

#include "types.h"

// ===========================================================================
//  ABU Auto Thailand V3 - Master brain  (Teensy 4.1)
//
//  >>> EVERY TUNING NUMBER IN THE MASTER FIRMWARE LIVES IN THIS FILE. <<<
//
//  Serial map
//    Serial   USB          Serial Monitor / command console
//    Serial1  460800       Sensor hub (LiDAR + TFMini + relays + switches)
//    Serial2  115200       Wheel driver STM32  (T,FL,FR,RL,RR / S / Q)
//    Serial3  115200       Keypad panel F103   (mission code + '\n')
//    Serial6  115200       Arm slave STM32     (CMD,<seq>,<command>)
//    Serial7  115200       Lift controller STM32 (LP,<front>,<back> / LZ)
//    Wire     400 kHz      BNO085 IMU on SDA pin 18 / SCL pin 19
// ===========================================================================

// ------------------------------------------------------------- baud rate --
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t HUB_BAUD = 460800;
constexpr uint32_t WHEEL_BAUD = 115200;
constexpr uint32_t HUB_TIMEOUT_MS = 300;
constexpr uint32_t LIFT_BAUD = 115200;
constexpr uint32_t ARM_BAUD = 115200;

// Keypad panel link:
// Teensy 4.1 pin 15 (RX3) <- F103 PA9  (TX)
// Teensy 4.1 pin 14 (TX3) -> F103 PA10 (RX)     [wired, not used yet]
// Connect the GND of both boards together.
//
// >> MUST match F103_SERIAL_BAUD in include/slave_button/config.h. <<
//    Both boards have to change together or the link reads pure garbage.
//    115200 was chosen over the F103's old 921600 because it is what every
//    other slave uses and it is ~8x more tolerant of the motor/relay EMI
//    around the robot. One keypress per match makes the speed irrelevant.
constexpr uint32_t BUTTON_BAUD = 115200;

// External arm link:
// Teensy 4.1 Serial6 TX (pin 24) -> Arm Slave Serial1 RX (PA10)
// Teensy 4.1 Serial6 RX (pin 25) <- Arm Slave Serial1 TX (PA9)
// Connect the GND of both boards together.
constexpr uint32_t ARM_DEFAULT_TIMEOUT_MS = 20000;
constexpr uint32_t ARM_ACK_TIMEOUT_MS = 300;
constexpr uint8_t ARM_MAX_RETRIES = 3;
constexpr float ARM_MIN_DEG = 0.0f;
constexpr float ARM_MAX_DEG = 180.0f;

// --------------------------------------------------------- loop periods ---
constexpr uint32_t LIFT_MOVE_TIMEOUT_MS = 15000;
constexpr uint32_t COMMAND_PERIOD_MS = 50;
constexpr uint32_t POSITION_PERIOD_MS = 20;
constexpr uint32_t POSE_PRINT_PERIOD_MS = 100;
constexpr uint32_t LIDAR_PRINT_PERIOD_MS = 250;
constexpr uint32_t FEEDBACK_TIMEOUT_MS = 500;
constexpr uint32_t GYRO_TIMEOUT_MS = 500;
constexpr uint32_t GYRO_REPORT_INTERVAL_US = 10000;

// --------------------------------------------------------------- LiDAR ---
// The RPLiDAR hangs off the HUB, which does all the scan processing and
// sends the finished box/wall detection in the "HUB,..." packet.
// >> Detection thresholds (box width, cluster gap, filter alpha, ...) are
//    tuned in the HUB firmware: src/slave_hub/config.cpp <<
// Only the Master-side reaction timing lives here.
constexpr uint32_t LIDAR_CONTROL_PERIOD_MS = 50;
constexpr uint32_t LIDAR_LOST_TIMEOUT_MS = 1000;

// ----------------------------------------------------- box parking gains --
// Low-speed box parking gains. Distance/offset errors are in millimetres.
constexpr float PARK_KP_DISTANCE = 1.0f;
constexpr float PARK_KP_OFFSET = 1.0f;
constexpr float PARK_KP_ANGLE = 2.0f;
constexpr float PARK_MAX_VX = 0.35f;
constexpr float PARK_MAX_VY = 0.35f;
constexpr float PARK_MAX_WZ = 1.2f;
constexpr float PARK_DISTANCE_TOL_MM = 20.0f;
constexpr float PARK_OFFSET_TOL_MM = 20.0f;
constexpr float PARK_ANGLE_TOL_DEG = 2.0f;
constexpr uint32_t PARK_DONE_HOLD_MS = 500;
constexpr float PARK_DISTANCE_DEADBAND_MM = 10.0f;
constexpr float PARK_OFFSET_DEADBAND_MM = 12.0f;
constexpr float PARK_ANGLE_DEADBAND_DEG = 1.2f;
constexpr float PARK_MIN_VX = 0.06f;
constexpr float PARK_MIN_VY = 0.06f;
constexpr float PARK_MIN_WZ = 0.08f;
constexpr float PARK_ACCEL_VX = 0.02f;
constexpr float PARK_ACCEL_VY = 0.02f;
constexpr float PARK_ACCEL_WZ = 0.05f;

// Change these signs if the LiDAR is mounted upside-down/backwards or the
// robot moves in the opposite direction during the first low-speed test.
constexpr float PARK_VX_SIGN = 1.0f;
constexpr float PARK_VY_SIGN = 1.0f;
constexpr float PARK_WZ_SIGN = 1.0f;

// ----------------------------------------------------------------- IMU ----
constexpr int8_t BNO08X_RESET = -1;

// ----------------------------------------------- chassis and kinematics ---
constexpr float COUNTS_PER_REV = 691.0f;
constexpr float WHEEL_RADIUS_M = 0.076f;
constexpr float LX_M = 0.1725f;
constexpr float LY_M = 0.2150f;
constexpr float L_SUM_M = LX_M + LY_M;
constexpr float MAX_RPM = 420.0f;

// Velocity ramp for V (local) and G (global) commands.
// Acceleration/deceleration units are m/s^2 and rad/s^2.
constexpr float LINEAR_ACCEL = 0.40f;
constexpr float LINEAR_DECEL = 0.60f;
constexpr float ANGULAR_ACCEL = 1.20f;
constexpr float ANGULAR_DECEL = 1.50f;
constexpr float RAMP_ZERO_EPSILON = 0.0001f;

// ------------------------------------- position controller (world frame) --
constexpr float KP_POSITION = 2.50f;
constexpr float KD_POSITION = 0.2f;//0.2
constexpr float KP_YAW = 3.20f;
constexpr float KD_YAW = 0.2f;//0.2
constexpr float MAX_POSITION_SPEED = 1.0f;
constexpr float MAX_POSITION_WZ = 0.80f;
constexpr float POSITION_TOLERANCE_M = 0.05f;
constexpr float YAW_TOLERANCE_RAD = 3.0f * DEG_TO_RAD;
constexpr uint8_t POSITION_DONE_COUNT = 10;

// ----------------------------------------------------- serial RX buffers --
constexpr size_t USB_RX_SIZE = 96;
constexpr size_t SLAVE_RX_SIZE = 160;
constexpr size_t HUB_RX_SIZE = 192;
constexpr size_t LIFT_RX_SIZE = 96;
constexpr size_t BUTTON_RX_SIZE = 32;

// ------------------------------------------------------- keypad mission code
// The keypad sends the same code REPEAT_COUNT times, REPEAT_INTERVAL_MS apart
// (5 x 20 ms in src/slave_button/main.cpp -- that is a contract between the
// two boards, neither side changes it alone). The Master collects whatever
// arrives inside this window and runs the code that appears most often, so a
// single corrupted frame is outvoted instead of driving the robot somewhere.
// The window also absorbs the repeats, which is why there is no separate
// "ignore repeats for N ms" timer anywhere.
//
// 5 frames spread over ~80 ms; 150 ms leaves room for the last one to land.
// Cost is ~150 ms of delay before a mission starts, once per match.
constexpr uint32_t BUTTON_VOTE_WINDOW_MS = 150;

// Distinct codes tracked inside one window. Reaching this many different
// codes in 150 ms is not a vote any more, it is a broken cable, so the whole
// window is thrown away when it overflows.
constexpr uint8_t BUTTON_VOTE_MAX_CANDIDATES = 8;

// Accepted code lengths, keyed by the first digit (the Mode).
// A table rather than if/else so a future Mode 2 is one line, not a branch.
//   Mode 0 -> field(1) + line(1) + box(4) + mode(1) = 7   e.g. 0120011
//   Mode 1 -> field(1) + row(1)  + step(1) + mode(1) = 4   e.g. 1130
struct MissionCodeLength
{
  char modeDigit;
  uint8_t length;
};

constexpr MissionCodeLength MISSION_CODE_LENGTHS[] =
{
  { '0', 7 },
  { '1', 4 },
};

// Extra hardware RX memory for the high-rate HUB stream on Teensy 4.1.
// Serial1.addMemoryForRead() must be called before Serial1.begin().
constexpr size_t HUB_SERIAL_RX_MEMORY_SIZE = 1024;

// ------------------------------------------------- Hub servo end stops ---
// The Master clamps "ARM n" / "SPIN n" to these before sending them to the
// Hub. They MUST match ARM_MAX_DEG / SPIN_MAX_DEG in
// include/slave_hub/config.h, otherwise a command is silently capped here
// and the extra travel never reaches the servo.
constexpr uint8_t HUB_ARM_MAX_DEG = 80;
constexpr uint8_t HUB_SPIN_MAX_DEG = 133;

// --------------------------------------------------- TFMini-S behaviour ---
// TFMini-S virtual laser mapping:
// TF1 -> Laser4, TF2 -> Laser3, TF3 -> Laser2
// Adjust these distances independently to suit the mechanism.
constexpr uint8_t HUB_LASER1 = 9;   // Virtual-only identifier
constexpr uint8_t HUB_LASER2 = 10;  // Virtual-only identifier
constexpr uint8_t HUB_LASER3 = 11;  // Virtual-only identifier
constexpr uint8_t HUB_LASER4 = 12;  // Virtual-only identifier
constexpr uint16_t TF1_MIN_CM = 5;
constexpr uint16_t TF1_MAX_CM = 15;
constexpr uint16_t TF2_MIN_CM = 5;
constexpr uint16_t TF2_MAX_CM = 15;
constexpr uint16_t TF3_MIN_CM = 5;
constexpr uint16_t TF3_MAX_CM = 15;
constexpr uint16_t TF4_MIN_CM = 5;
constexpr uint16_t TF4_MAX_CM = 15;

// TF3/TF4 edge detection must remain "no floor" for this many loop checks
// before that side stops. The step is armed only after both sensors have
// first seen the floor, so starting beyond the edge cannot trigger it.
constexpr uint8_t TF34_EDGE_CONFIRM_COUNT = 3;
constexpr uint8_t TF1_DETECT_CONFIRM_COUNT = 3;

// Sensor mounting for TF34_EDGE_FRONT_LIFT_STEP:
// TF3 controls the left wheels (FL/RL), TF4 controls the right wheels (FR/RR).
// If the sensors are physically mounted on the opposite sides, swap these.
constexpr bool TF3_EDGE_IS_LEFT_SIDE = true;

// LASER3_FORWARD_STEP stops when TF2 is closer than this distance.
constexpr uint16_t TF2_FORWARD_STOP_CM = 15;

// LASER4_FORWARD_STEP stops when TF1 is closer than this distance.
constexpr uint16_t TF1_FORWARD_STOP_CM = 20;

// TF1 condition used while descending from a step.
// TF1 is TRUE while it still sees the floor at or below this distance.
// The descend step is armed only after TRUE has been seen, then stops after
// TF1 changes to FALSE (distance greater than this threshold) for 3 updates.
constexpr uint16_t TF1_DESCEND_FLOOR_MAX_CM = 10;

// After TF1 changes from floor (TRUE) to no floor (FALSE), keep moving
// slowly for a short time so the rear wheel on the opposite side also
// clears the edge before the back lift is lowered.
constexpr float TF1_DESCEND_CLEARANCE_SPEED_MPS = 0.12f;
constexpr uint32_t TF1_DESCEND_CLEARANCE_MS = 500;

// ---------------------------------------------------------------- lift ----
constexpr int32_t LIFT_TOLERANCE = 100;
constexpr uint8_t LIFT_CONFIRM_REQUIRED = 2;

// -------------------------------------- higher-step alignment (HIGH ...) --
constexpr float HIGH_APPROACH_SPEED_MPS = 0.4f;
constexpr float HIGH_ONE_SIDE_SPEED_MPS = 0.2f;
constexpr float HIGH_GYRO_KP = 2.0f;
constexpr float HIGH_MAX_WZ = 0.25f;
constexpr uint32_t HIGH_CONFIRM_MS = 150;
constexpr float HIGH_BACKOFF_SPEED_MPS = 0.3f;
constexpr uint32_t HIGH_BACKOFF_MS = 1000;
constexpr uint32_t HIGH_TIMEOUT_MS = 10000;
constexpr int32_t HIGH_START_LIFT_FRONT_PULSE = 100;
constexpr int32_t HIGH_START_LIFT_BACK_PULSE = 200;
constexpr int32_t HIGH_DEFAULT_AFTER_LIFT_FRONT_PULSE = 3800;
constexpr int32_t HIGH_DEFAULT_AFTER_LIFT_BACK_PULSE = 3900;
constexpr uint32_t HIGH_LIFT_TIMEOUT_MS = 10000;
constexpr int32_t HIGH_SOFT_LIFT_STEP_PULSE = 200;
constexpr uint32_t HIGH_SOFT_LIFT_STEP_MS = 50;
constexpr float HIGH_LIDAR_TARGET_DISTANCE_MM = 380.0f;
constexpr uint32_t HIGH_LIDAR_TIMEOUT_MS = 2000;
constexpr float HIGH_STEP_CONTACT_SPEED_MPS = 0.3f;
constexpr uint32_t HIGH_STEP_CONTACT_DRIVE_MS = 2000;
constexpr float HIGH_NO_BOX_FORWARD_SPEED_MPS = 0.3f;
constexpr uint32_t HIGH_NO_BOX_FORWARD_TIMEOUT_MS = 15000;

// ------------------------------------------------------------- mission ----
constexpr uint32_t DEFAULT_MOVE_TIMEOUT_MS = 30000;

constexpr uint32_t LASER5_ARM_VERIFY_WAIT_MS   = 500;
constexpr uint32_t LASER5_GRIPPER_OPEN_WAIT_MS = 500;
constexpr uint32_t LASER5_ARM_RETURN_WAIT_MS   = 500;

// --------------------------------------------------------- hub buttons ----
constexpr uint32_t HUB_BUTTON_DEBOUNCE_MS = 40;
constexpr uint8_t GRIPPER_RELAY_NUMBER = 4;

// SW_Y reset pulses Relay 4 (gripper open) for this long, then releases it.
constexpr uint32_t YELLOW_RELAY4_PULSE_MS = 500;
