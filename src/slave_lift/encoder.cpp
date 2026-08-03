#include "encoder.h"

static volatile int32_t frontEncoderCount = 0;
static volatile int32_t backEncoderCount = 0;

static void frontEncoderA_ISR() {
  if (digitalRead(FRONT_ENCODER_A) != digitalRead(FRONT_ENCODER_B)) {
    frontEncoderCount++;
  } else {
    frontEncoderCount--;
  }
}

static void frontEncoderB_ISR() {
  if (digitalRead(FRONT_ENCODER_A) == digitalRead(FRONT_ENCODER_B)) {
    frontEncoderCount++;
  } else {
    frontEncoderCount--;
  }
}

static void backEncoderA_ISR() {
  if (digitalRead(BACK_ENCODER_A) != digitalRead(BACK_ENCODER_B)) {
    backEncoderCount++;
  } else {
    backEncoderCount--;
  }
}

static void backEncoderB_ISR() {
  if (digitalRead(BACK_ENCODER_A) == digitalRead(BACK_ENCODER_B)) {
    backEncoderCount++;
  } else {
    backEncoderCount--;
  }
}

void attachEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(FRONT_ENCODER_A), frontEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONT_ENCODER_B), frontEncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_ENCODER_A), backEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BACK_ENCODER_B), backEncoderB_ISR, CHANGE);
}

int32_t readFrontEncoder() {
  noInterrupts();
  const int32_t value = frontEncoderCount;
  interrupts();
  return value;
}

void resetFrontEncoder() {
  noInterrupts();
  frontEncoderCount = 0;
  interrupts();
}

int32_t readBackEncoder() {
  noInterrupts();
  const int32_t value = backEncoderCount;
  interrupts();
  return value;
}

void resetBackEncoder() {
  noInterrupts();
  backEncoderCount = 0;
  interrupts();
}

void resetBothEncoders() {
  noInterrupts();
  frontEncoderCount = 0;
  backEncoderCount = 0;
  interrupts();
}
