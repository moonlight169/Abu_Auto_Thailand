#pragma once

#include "config.h"

// Quadrature encoder counting for both arm axes.

void attachEncoderInterrupts();

int32_t readTopEncoder();
void resetTopEncoder();

int32_t readBottomEncoder();
void resetBottomEncoder();
