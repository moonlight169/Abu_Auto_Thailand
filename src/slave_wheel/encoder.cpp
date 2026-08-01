#include "encoder.h"

volatile int32_t encoderCount[WHEEL_COUNT] = {0, 0, 0, 0};
int32_t previousEncoderCount[WHEEL_COUNT] = {0, 0, 0, 0};

static int32_t rpmDeltaBuffer[WHEEL_COUNT][RPM_FILTER_SIZE] = {};
static int32_t rpmDeltaSum[WHEEL_COUNT] = {0, 0, 0, 0};
static uint8_t rpmBufferIndex[WHEEL_COUNT] = {0, 0, 0, 0};
static uint8_t rpmValidSamples[WHEEL_COUNT] = {0, 0, 0, 0};

float rawRPM[WHEEL_COUNT] = {0, 0, 0, 0};
float actualRPM[WHEEL_COUNT] = {0, 0, 0, 0};

static void encoderUpdate(uint8_t wheel, bool interruptFromA)
{
  const bool stateA = digitalRead(ENCODER_PIN_A[wheel]);
  const bool stateB = digitalRead(ENCODER_PIN_B[wheel]);

  int8_t direction;
  if (interruptFromA)
    direction = (stateA == stateB) ? 1 : -1;
  else
    direction = (stateA != stateB) ? 1 : -1;

  encoderCount[wheel] += direction * ENCODER_DIRECTION[wheel];
}

static void encoderFL_A_ISR() { encoderUpdate(FL, true); }
static void encoderFL_B_ISR() { encoderUpdate(FL, false); }
static void encoderFR_A_ISR() { encoderUpdate(FR, true); }
static void encoderFR_B_ISR() { encoderUpdate(FR, false); }
static void encoderRL_A_ISR() { encoderUpdate(RL, true); }
static void encoderRL_B_ISR() { encoderUpdate(RL, false); }
static void encoderRR_A_ISR() { encoderUpdate(RR, true); }
static void encoderRR_B_ISR() { encoderUpdate(RR, false); }

void attachEncoderInterrupts()
{
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A[FL]), encoderFL_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B[FL]), encoderFL_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A[FR]), encoderFR_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B[FR]), encoderFR_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A[RL]), encoderRL_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B[RL]), encoderRL_B_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A[RR]), encoderRR_A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B[RR]), encoderRR_B_ISR, CHANGE);
}

void resetRPMFilter(uint8_t wheel)
{
  rpmDeltaSum[wheel] = 0;
  rpmBufferIndex[wheel] = 0;
  rpmValidSamples[wheel] = 0;

  for (uint8_t i = 0; i < RPM_FILTER_SIZE; i++)
    rpmDeltaBuffer[wheel][i] = 0;

  noInterrupts();
  previousEncoderCount[wheel] = encoderCount[wheel];
  interrupts();

  rawRPM[wheel] = 0.0f;
  actualRPM[wheel] = 0.0f;
}

void updateRPM(uint8_t wheel)
{
  noInterrupts();
  const int32_t currentCount = encoderCount[wheel];
  interrupts();

  const int32_t deltaCount = currentCount - previousEncoderCount[wheel];
  previousEncoderCount[wheel] = currentCount;

  rawRPM[wheel] =
      ((float)deltaCount / COUNTS_PER_REV) * (60.0f / CONTROL_DT);

  rpmDeltaSum[wheel] -=
      rpmDeltaBuffer[wheel][rpmBufferIndex[wheel]];
  rpmDeltaBuffer[wheel][rpmBufferIndex[wheel]] = deltaCount;
  rpmDeltaSum[wheel] += deltaCount;

  rpmBufferIndex[wheel]++;
  if (rpmBufferIndex[wheel] >= RPM_FILTER_SIZE)
    rpmBufferIndex[wheel] = 0;

  if (rpmValidSamples[wheel] < RPM_FILTER_SIZE)
    rpmValidSamples[wheel]++;

  const float measurementTime =
      CONTROL_DT * (float)rpmValidSamples[wheel];

  actualRPM[wheel] =
      ((float)rpmDeltaSum[wheel] / COUNTS_PER_REV) *
      (60.0f / measurementTime);
}
