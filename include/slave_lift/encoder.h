#pragma once

#include "config.h"

// Quadrature encoder counting for both lift columns.
// Positive counts are upward.

void attachEncoderInterrupts();

int32_t readFrontEncoder();
void resetFrontEncoder();

int32_t readBackEncoder();
void resetBackEncoder();

void resetBothEncoders();
