#pragma once

#include "config.h"

// ---------------------------------------------------------------------------
//  Hub push buttons - one action per press, with debounce.
//    SW_G: start the currently selected mission.
//    SW_B: abort motion, reset odometry/gyro, then Arm CLEAR -> HOME.
//    SW_A: start the currently selected mission.
//    SW_Y: reset gyro yaw and odometry.
// ---------------------------------------------------------------------------

void updateHubButtons();

// Available hook, currently not bound to a button.
void toggleGripper();
