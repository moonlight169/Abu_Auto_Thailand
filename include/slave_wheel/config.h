#pragma once

#include <Arduino.h>

// ===========================================================================
//  Wheel driver - tuning and hardware map (Blackpill STM32F411CE)
//
//  Edit numbers HERE. The pin / gain tables live in config.cpp so every
//  wheel-dependent value stays in exactly one place.
// ===========================================================================

enum WheelIndex : uint8_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
  WHEEL_COUNT = 4
};

// ------------------------------------------------ tables -> config.cpp ----
extern const char *WHEEL_NAME[WHEEL_COUNT];

extern const uint32_t MOTOR_PIN_A[WHEEL_COUNT];
extern const uint32_t MOTOR_PIN_B[WHEEL_COUNT];
extern const uint32_t ENCODER_PIN_A[WHEEL_COUNT];
extern const uint32_t ENCODER_PIN_B[WHEEL_COUNT];

// Flip a sign here if a wheel spins or counts the wrong way.
extern int8_t MOTOR_DIRECTION[WHEEL_COUNT];
extern int8_t ENCODER_DIRECTION[WHEEL_COUNT];

// Speed PID gains, one set per wheel.
extern float kp[WHEEL_COUNT];
extern float ki[WHEEL_COUNT];
extern float kd[WHEEL_COUNT];

// ----------------------------------------------------------- mechanics ----
constexpr float WHEEL_RADIUS_M = 0.076f;
constexpr float LX_M = 0.1725f;
constexpr float LY_M = 0.2150f;
constexpr float L_SUM_M = LX_M + LY_M;

constexpr float COUNTS_PER_REV = 691.0f;
constexpr float MAX_RPM = 420.0f;

constexpr int16_t PWM_MIN = -255;
constexpr int16_t PWM_MAX = 255;

// -------------------------------------------------------------- timing ----
constexpr uint32_t CONTROL_PERIOD_US = 10000UL;
constexpr float CONTROL_DT = 0.010f;
constexpr uint32_t STATUS_PERIOD_MS = 100UL;
constexpr uint32_t SERIAL1_BAUD = 115200UL;
constexpr uint32_t FEEDBACK_PERIOD_MS = 20UL;
constexpr uint32_t SERIAL1_TIMEOUT_MS = 500UL;
constexpr uint8_t RPM_FILTER_SIZE = 5;
constexpr uint8_t SERIAL1_RX_BUFFER_SIZE = 80;
