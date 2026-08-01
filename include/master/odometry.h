#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Mecanum odometry from the wheel encoder counts, fused with the BNO085 yaw
//  when the IMU is online.
// ---------------------------------------------------------------------------

void resetOdometry();
void updateOdometry(const int32_t newCount[WHEEL_COUNT]);

void printRobotPose();
void updatePosePrint();
