#include "encoder.h"

static volatile int32_t topEncoderCount = 0;
static volatile int32_t bottomEncoderCount = 0;

static void topEncoderA_ISR() {
  if (digitalRead(TOP_ENCODER_A) != digitalRead(TOP_ENCODER_B)) {
    topEncoderCount++;
  } else {
    topEncoderCount--;
  }
}

static void topEncoderB_ISR() {
  if (digitalRead(TOP_ENCODER_A) == digitalRead(TOP_ENCODER_B)) {
    topEncoderCount++;
  } else {
    topEncoderCount--;
  }
}

static void bottomEncoderA_ISR() {
  if (digitalRead(BOTTOM_ENCODER_A) != digitalRead(BOTTOM_ENCODER_B)) {
    bottomEncoderCount++;
  } else {
    bottomEncoderCount--;
  }
}

static void bottomEncoderB_ISR() {
  if (digitalRead(BOTTOM_ENCODER_A) == digitalRead(BOTTOM_ENCODER_B)) {
    bottomEncoderCount++;
  } else {
    bottomEncoderCount--;
  }
}

void attachEncoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(TOP_ENCODER_A), topEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TOP_ENCODER_B), topEncoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_ENCODER_A), bottomEncoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_ENCODER_B), bottomEncoderB_ISR, CHANGE);
}

int32_t readTopEncoder() {
  noInterrupts();
  int32_t value = topEncoderCount;
  interrupts();
  return value;
}

void resetTopEncoder() {
  noInterrupts();
  topEncoderCount = 0;
  interrupts();
}

int32_t readBottomEncoder() {
  noInterrupts();
  int32_t value = bottomEncoderCount;
  interrupts();
  return value;
}

void resetBottomEncoder() {
  noInterrupts();
  bottomEncoderCount = 0;
  interrupts();
}
