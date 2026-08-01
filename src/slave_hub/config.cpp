#include "config.h"

HardwareSerial &master = Serial1;
HardwareSerial &lidar = Serial8;
HardwareSerial &tfmini1 = Serial2;
HardwareSerial &tfmini2 = Serial6;
HardwareSerial &tfmini3 = Serial7;
HardwareSerial &tfmini4 = Serial3;

float TARGET_DIST_MM = 450.0f;
float VIEW_HALF_ANGLE_DEG = 55.0f;
float DETECT_MIN_DIST_MM = 150.0f;
float DETECT_MAX_DIST_MM = 1500.0f;
float BOX_MIN_WIDTH_MM = 250.0f;
float BOX_MAX_WIDTH_MM = 1200.0f;
int MIN_BOX_POINTS = 8;
float MAX_CLUSTER_GAP_MM = 160.0f;
float MAX_CLUSTER_ANGLE_GAP_DEG = 5.0f;
float MAX_LINE_ERROR_MM = 35.0f;
float MAX_WALL_LINE_ERROR_MM = 45.0f;
int MIN_WALL_POINTS = 12;
uint8_t MIN_LIDAR_QUALITY = 1;
float OFFSET_SIGN = 1.0f;
float ANGLE_SIGN = 1.0f;
float FILTER_ALPHA = 0.45f;
int MAX_LOST_COUNT = 5;
