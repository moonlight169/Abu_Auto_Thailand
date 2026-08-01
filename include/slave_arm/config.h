#pragma once

#include <Arduino.h>

// ===========================================================================
//  ABU 2026 - Slave Arm, dual PID  (Blackpill STM32F411CE)
//
//  Bottom and Top arm:
//    - independent encoder PID position control, 0..180 degrees
//    - position 0 degree is set at each FRONT limit
//    - Top homing starts only after Bottom FRONT has been confirmed
//
//  All limit switches are ACTIVE LOW and use INPUT_PULLUP.
//
//  Everything tunable lives in this file.
// ===========================================================================

// ------------------------------------------------------------ pin map -----
constexpr uint32_t TOP_MOTOR_A       = PB0;
constexpr uint32_t TOP_MOTOR_B       = PB1;
constexpr uint32_t BOTTOM_MOTOR_A    = PA7;
constexpr uint32_t BOTTOM_MOTOR_B    = PA6;

constexpr uint32_t TOP_ENCODER_A     = PB12;
constexpr uint32_t TOP_ENCODER_B     = PB13;
constexpr uint32_t BOTTOM_ENCODER_A  = PB15;
constexpr uint32_t BOTTOM_ENCODER_B  = PB14;

constexpr uint32_t TOP_LIMIT_FRONT    = PB4;
constexpr uint32_t TOP_LIMIT_BACK     = PB5;
constexpr uint32_t BOTTOM_LIMIT_FRONT = PB6;
constexpr uint32_t BOTTOM_LIMIT_BACK  = PB7;

// --------------------------------------------- direction and speed --------
// Original convention:
//   Bottom  negative = FRONT, positive = BACK
//   Top     positive = FRONT, negative = BACK
constexpr int BOTTOM_HOME_PWM = -200;
constexpr int TOP_HOME_PWM    =  120;

constexpr int BOTTOM_MAX_PWM      = 255;
constexpr int BOTTOM_MIN_MOVE_PWM = 35;
constexpr int TOP_MAX_PWM         = 200;
constexpr int TOP_MIN_MOVE_PWM    = 35;

// Encoder scales. Change the sign if position counts backward.
constexpr float BOTTOM_PULSE_PER_DEG = 910.0f;
constexpr float TOP_PULSE_PER_DEG = -890.0f;
constexpr float BOTTOM_MIN_DEG = 0.0f;
constexpr float BOTTOM_MAX_DEG = 180.0f;
constexpr float TOP_MIN_DEG = 0.0f;
constexpr float TOP_MAX_DEG = 180.0f;
constexpr float TOP_HOME_READY_DEG = 100.0f;

// ------------------------------------------------- position controller ----
// Start here, then tune on the real arm.
constexpr float BOTTOM_KP = 5.000f;
constexpr float BOTTOM_KI = 0.000f;
constexpr float BOTTOM_KD = 0.000f;
constexpr float BOTTOM_TOLERANCE_DEG = 1.0f;
constexpr float BOTTOM_RELEASE_DEG   = 2.0f;
constexpr float TOP_KP = 5.000f;  // PWM per pulse
constexpr float TOP_KI = 0.200f;
constexpr float TOP_KD = 0.000f;
constexpr float TOP_TOLERANCE_DEG = 1.0f;
constexpr float TOP_RELEASE_DEG   = 2.0f;
constexpr uint32_t CONTROL_PERIOD_MS = 10;

// -------------------------------------------------------------- timing ----
constexpr uint32_t MOVE_TIMEOUT_MS = 50000;
constexpr uint32_t STATUS_PERIOD_MS = 500;
constexpr uint32_t LIMIT_STREAM_PERIOD_MS = 200;
constexpr uint32_t MASTER_BAUD = 115200;

// Arm link:
//   Slave Serial1 RX PA10 <- Teensy 4.1 Serial6 TX pin 24
//   Slave Serial1 TX PA9  -> Teensy 4.1 Serial6 RX pin 25
//   Connect the GND of both boards together.
constexpr uint32_t LIMIT_DEBOUNCE_MS = 40;
constexpr uint32_t BOTTOM_HOME_CONFIRM_MS = 200;
constexpr int BOTTOM_HOME_RELEASE_PWM = 120;
constexpr size_t MASTER_RX_BUFFER_SIZE = 96;
constexpr size_t MASTER_REPLY_BUFFER_SIZE = 128;
constexpr uint8_t CSV_PROTOCOL_VERSION = 2;

// ------------------------------------------------------------- types ------
enum ArmState : uint8_t {
  IDLE,
  HOME_BOTTOM_RELEASE,
  HOME_BOTTOM_FIRST,
  HOME_BOTTOM_CONFIRM,
  HOME_TOP_FRONT,
  HOME_TOP_TO_READY,
  BOTTOM_POSITION_MOVE,
  TOP_POSITION_MOVE,
  TEST_BOTTOM_PWM,
  TEST_TOP_PWM,
  FAULT
};

struct DebouncedLimit {
  uint32_t pin;
  bool rawPressed;
  bool stablePressed;
  uint32_t changedMs;
};

enum MasterJob : uint8_t {
  MASTER_JOB_NONE,
  MASTER_JOB_HOME,
  MASTER_JOB_BOTTOM,
  MASTER_JOB_TOP,
  MASTER_JOB_POSE_BOTTOM,
  MASTER_JOB_POSE_TOP
};
