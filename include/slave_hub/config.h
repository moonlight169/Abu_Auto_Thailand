#pragma once

#include <Arduino.h>

// ===========================================================================
//  ABU Auto Thailand V3 - Sensor hub  (Teensy 4.1)
//
//  Everything tunable lives in this file. The values that stay adjustable at
//  runtime (LiDAR thresholds, filter alpha, ...) are plain variables defined
//  in config.cpp; the fixed ones are constexpr.
// ===========================================================================

// ------------------------------------------------------- serial ports -----
//  master  Serial1 @ 460800   RX1 = 0,  TX1 = 1   -> Teensy Master
//  lidar   Serial8 @ 460800   RX8 = 34, TX8 = 35  -> RPLiDAR
//  tfmini1 Serial2 @ 115200
//  tfmini2 Serial6 @ 115200
//  tfmini3 Serial7 @ 115200
//  tfmini4 Serial3 @ 115200   RX3 = 15, TX3 = 14
extern HardwareSerial &master;
extern HardwareSerial &lidar;
extern HardwareSerial &tfmini1;
extern HardwareSerial &tfmini2;
extern HardwareSerial &tfmini3;
extern HardwareSerial &tfmini4;

constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t MASTER_BAUD = 460800;
constexpr uint32_t LIDAR_BAUD = 460800;
constexpr uint32_t TFMINI_BAUD = 115200;
constexpr uint32_t TFMINI_TIMEOUT_MS = 300;

// ===================== OUTPUT =====================
#define relay1     2
#define relay2     3
#define relay3     4
#define relay4     5
#define arm_servo  6
#define spin_servo 10

// ====================== INPUT =====================
#define L_SW_Front 12
#define R_SW_Front 17
#define LDR1       13
#define LDR2       18

#define laser5_sen 19
#define SW_Green   20
#define SW_Blue    21
#define SW_Red     22
#define SW_Yellow  23

// Relay LOW = ON, Input LOW = ACTIVE.
constexpr bool RELAY_ACTIVE_HIGH = false;
constexpr uint32_t INPUT_DEBOUNCE_MS = 30;

// Mechanical safe ranges.
// NOTE: the Master clamps "ARM n" / "SPIN n" to the same limits before it
// sends them, so keep HUB_ARM_MAX_DEG / HUB_SPIN_MAX_DEG in
// include/master/config.h in sync with these two lines.
constexpr int ARM_MIN_DEG = 0;
constexpr int ARM_MAX_DEG = 80;
constexpr int SPIN_MIN_DEG = 0;
constexpr int SPIN_MAX_DEG = 133;

constexpr uint8_t TFMINI_COUNT = 4;

// ------------------------------------------------- LiDAR detection --------
//  Runtime-adjustable from the console (D / A / T commands). Defined in
//  config.cpp.
extern float TARGET_DIST_MM;
extern float VIEW_HALF_ANGLE_DEG;
extern float DETECT_MIN_DIST_MM;
extern float DETECT_MAX_DIST_MM;
extern float BOX_MIN_WIDTH_MM;
extern float BOX_MAX_WIDTH_MM;
extern int MIN_BOX_POINTS;
extern float MAX_CLUSTER_GAP_MM;
extern float MAX_CLUSTER_ANGLE_GAP_DEG;
extern float MAX_LINE_ERROR_MM;
extern float MAX_WALL_LINE_ERROR_MM;
// WALL mode does not require a fixed physical length. A wall is accepted
// when enough consecutive front points form a sufficiently straight line.
extern int MIN_WALL_POINTS;
extern uint8_t MIN_LIDAR_QUALITY;
extern float OFFSET_SIGN;
extern float ANGLE_SIGN;
extern float FILTER_ALPHA;
extern int MAX_LOST_COUNT;

constexpr int MAX_SCAN_POINTS = 320;
constexpr int BOX_BAR_WIDTH = 41;
constexpr float BOX_BAR_HALF_WIDTH_MM = 1000.0f;

// -------------------------------------------------------------- types -----
enum DetectionMode : uint8_t {
  MODE_BOX = 1,
  MODE_WALL = 2
};

struct TfminiData {
  uint8_t frame[9];
  uint8_t index;
  uint16_t distanceCm;
  uint16_t strength;
  uint32_t lastFrameMs;
  bool valid;
};

struct ScanPoint {
  float angle;
  float x;
  float y;
  float dist;
};

struct BoxResult {
  bool found;
  float angleDeg;
  float distanceMm;
  float offsetMm;
  float widthMm;
  float lineErrorMm;
  int points;
};

struct InputChannel {
  const char *name;
  uint8_t pin;
  bool stableState;
  bool lastRawState;
  uint32_t changedTime;
};
