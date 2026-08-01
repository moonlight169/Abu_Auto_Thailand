#include "config.h"

const char *WHEEL_NAME[WHEEL_COUNT] = {"FL", "FR", "RL", "RR"};

const uint32_t MOTOR_PIN_A[WHEEL_COUNT] = {PA1, PA2, PA6, PB0};
const uint32_t MOTOR_PIN_B[WHEEL_COUNT] = {PA0, PA3, PA7, PB1};

const uint32_t ENCODER_PIN_A[WHEEL_COUNT] = {PB6, PB5, PB14, PB13};
const uint32_t ENCODER_PIN_B[WHEEL_COUNT] = {PB7, PB4, PB15, PB12};

int8_t MOTOR_DIRECTION[WHEEL_COUNT]   = {1, 1, 1, 1};
int8_t ENCODER_DIRECTION[WHEEL_COUNT] = {1, 1, 1, 1};

float kp[WHEEL_COUNT] = {1.3f, 1.0f, 1.3f, 1.0f};
float ki[WHEEL_COUNT] = {1.5f, 1.5f, 1.5f, 1.5f};
float kd[WHEEL_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
