#include "motor.h"

void initializeMotorPins() {
  analogWriteResolution(PWM_RESOLUTION_BITS);
  analogWriteFrequency(PWM_FREQUENCY_HZ);

  pinMode(FRONT_MOTOR_A, OUTPUT);
  pinMode(FRONT_MOTOR_B, OUTPUT);
  pinMode(BACK_MOTOR_A, OUTPUT);
  pinMode(BACK_MOTOR_B, OUTPUT);

  pinMode(FRONT_ENCODER_A, INPUT_PULLUP);
  pinMode(FRONT_ENCODER_B, INPUT_PULLUP);
  pinMode(BACK_ENCODER_A, INPUT_PULLUP);
  pinMode(BACK_ENCODER_B, INPUT_PULLUP);

  pinMode(FRONT_LIMIT_TOP, INPUT_PULLUP);
  pinMode(FRONT_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(BACK_LIMIT_TOP, INPUT_PULLUP);
  pinMode(BACK_LIMIT_BOTTOM, INPUT_PULLUP);
}

bool limitActive(uint32_t pin) {
  return digitalRead(pin) == LOW;
}

void motorWrite(uint32_t pinA, uint32_t pinB, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  if (pwm > 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else if (pwm < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void frontMotor(int pwm) {
  motorWrite(FRONT_MOTOR_A, FRONT_MOTOR_B, pwm);
}

void backMotor(int pwm) {
  motorWrite(BACK_MOTOR_A, BACK_MOTOR_B, pwm);
}
