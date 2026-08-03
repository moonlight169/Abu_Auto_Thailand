#pragma once

#include <Arduino.h>

// ===========================================================================
//  ABU 2026 - Slave Lift, dual-axis position PID  (Blackpill STM32F411CE)
//
//  Front and Back lift column:
//    - independent encoder PID position control, 0..*_PULSE_MAX pulses
//    - position 0 pulse is set at each BOTTOM limit while homing
//    - negative PWM drives a column UP, positive PWM drives it DOWN
//    - the downward direction is ramped and brakes before the target,
//      so the load never slams down
//
//  All limit switches are ACTIVE LOW and use INPUT_PULLUP.
//
//  Everything tunable lives in this file.
// ===========================================================================

// ------------------------------------------------------------ pin map -----
constexpr uint32_t FRONT_MOTOR_A = PB1;
constexpr uint32_t FRONT_MOTOR_B = PB0;
constexpr uint32_t BACK_MOTOR_A  = PA7;
constexpr uint32_t BACK_MOTOR_B  = PA6;

constexpr uint32_t FRONT_ENCODER_A = PB12;
constexpr uint32_t FRONT_ENCODER_B = PB13;
constexpr uint32_t BACK_ENCODER_A  = PB15;
constexpr uint32_t BACK_ENCODER_B  = PB14;

constexpr uint32_t FRONT_LIMIT_TOP    = PB5;
constexpr uint32_t FRONT_LIMIT_BOTTOM = PB4;
constexpr uint32_t BACK_LIMIT_TOP     = PB7;
constexpr uint32_t BACK_LIMIT_BOTTOM  = PB6;

// --------------------------------------------------------- master link ----
// Slave Serial1 RX PA10 <- Teensy 4.1 Serial7 TX
// Slave Serial1 TX PA9  -> Teensy 4.1 Serial7 RX
// Connect the GND of both boards together.
constexpr uint32_t MASTER_LINK_RX_PIN = PA10;
constexpr uint32_t MASTER_LINK_TX_PIN = PA9;
constexpr uint32_t MASTER_BAUD = 115200;

// --------------------------------------------------------- calibration ----
// Pulse count at the TOP of the stroke, measured after homing.
constexpr int32_t FRONT_PULSE_MAX = 3800;
constexpr int32_t BACK_PULSE_MAX  = 3900;
constexpr float FRONT_STROKE_MM = 400.0f;
constexpr float BACK_STROKE_MM  = 400.0f;

// ------------------------------------------------- position controller ----
constexpr int PWM_MAX = 255;
constexpr int HOME_PWM = 60;  // positive PWM moves downward

// Separate downward limits: the front column moves faster than the back.
// These limits affect only positive PWM (downward direction).
constexpr int FRONT_DOWN_PWM_MAX = 150;
constexpr int BACK_DOWN_PWM_MAX  = 150;
constexpr int DOWN_ACCEL_STEP = 1;  // PWM counts per control cycle
constexpr int DOWN_DECEL_STEP = 2;  // PWM counts per control cycle
constexpr int32_t DOWN_BRAKE_ZONE_PULSE = 600;
constexpr int32_t POSITION_TOLERANCE_PULSE = 10;
constexpr uint16_t REACHED_CONFIRM_CYCLES = 20;  // 20 x 10 ms = 200 ms

// Start-up gains. They can be retuned live from the console
// (p/i/d for both axes, fp/fi/fd and bp/bi/bd for one axis).
constexpr float FRONT_KP = 5.300f;
constexpr float FRONT_KI = 4.000f;
constexpr float FRONT_KD = 0.000f;
constexpr float BACK_KP = 2.300f;
constexpr float BACK_KI = 2.200f;
constexpr float BACK_KD = 0.000f;

// -------------------------------------------------------------- timing ----
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t PRINT_PERIOD_MS = 500;
constexpr uint32_t LIFT_POS_PERIOD_MS = 20;  // LIFT_POS stream to the Master

// ------------------------------------------------------- PWM generation ---
constexpr uint32_t PWM_RESOLUTION_BITS = 8;
constexpr uint32_t PWM_FREQUENCY_HZ = 12000;

// ------------------------------------------------------------- buffers ----
constexpr uint8_t COMMAND_BUFFER_SIZE = 48;
