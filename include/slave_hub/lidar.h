#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  RPLiDAR front-sector processing.
//
//  Points inside +/- VIEW_HALF_ANGLE_DEG are collected per revolution, split
//  into clusters, and each cluster is least-squares fitted to a line. The best
//  candidate becomes `result`, low-pass filtered with FILTER_ALPHA.
//
//  MODE_BOX  : a line of BOX_MIN..MAX_WIDTH near the robot centre.
//  MODE_WALL : the longest clean front plane, no fixed length required.
// ---------------------------------------------------------------------------

extern DetectionMode detectionMode;

extern BoxResult result;
extern bool filterReady;
extern int lostCount;

extern ScanPoint monitorPoints[MAX_SCAN_POINTS];
extern int monitorPointCount;

extern uint32_t validPacketCount;
extern uint32_t scanFrameCount;
extern uint32_t lastPacketTime;
extern uint32_t lastDetectionFrameTime;
extern int lastClusterCount;
extern int lastLineCount;

void beginLidar();
void readLidar();

void clearResult();
void resetDetectionFilter();
void setDetectionMode(DetectionMode newMode);
