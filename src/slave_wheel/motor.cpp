#include "motor.h"

void initializeMotorPins()
{
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    pinMode(MOTOR_PIN_A[wheel], OUTPUT);
    pinMode(MOTOR_PIN_B[wheel], OUTPUT);
    pinMode(ENCODER_PIN_A[wheel], INPUT_PULLUP);
    pinMode(ENCODER_PIN_B[wheel], INPUT_PULLUP);
  }
}

void setMotorPWM(uint8_t wheel, int16_t pwm)
{
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);
  pwm *= MOTOR_DIRECTION[wheel];

  if (pwm > 0)
  {
    analogWrite(MOTOR_PIN_A[wheel], pwm);
    analogWrite(MOTOR_PIN_B[wheel], 0);
  }
  else if (pwm < 0)
  {
    analogWrite(MOTOR_PIN_A[wheel], 0);
    analogWrite(MOTOR_PIN_B[wheel], -pwm);
  }
  else
  {
    analogWrite(MOTOR_PIN_A[wheel], 0);
    analogWrite(MOTOR_PIN_B[wheel], 0);
  }
}
