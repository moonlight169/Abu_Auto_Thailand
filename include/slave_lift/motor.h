#pragma once

#include "config.h"

// Low-level H-bridge output and raw limit-switch reads.
// Negative PWM drives a column UP, positive PWM drives it DOWN.
//
// Unlike the arm slave, the directional limit cut-off is NOT applied here:
// reaching a BOTTOM stop also re-zeroes that column's encoder and target, so
// the whole reaction lives together in lift_control.cpp.

void initializeMotorPins();

bool limitActive(uint32_t pin);

void motorWrite(uint32_t pinA, uint32_t pinB, int pwm);
void frontMotor(int pwm);
void backMotor(int pwm);
