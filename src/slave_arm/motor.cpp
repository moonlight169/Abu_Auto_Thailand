#include "motor.h"

void initializeMotorPins() {
  analogWriteResolution(8);
  analogWriteFrequency(5000);

  pinMode(TOP_MOTOR_A, OUTPUT);
  pinMode(TOP_MOTOR_B, OUTPUT);
  pinMode(BOTTOM_MOTOR_A, OUTPUT);
  pinMode(BOTTOM_MOTOR_B, OUTPUT);

  pinMode(TOP_ENCODER_A, INPUT_PULLUP);
  pinMode(TOP_ENCODER_B, INPUT_PULLUP);
  pinMode(BOTTOM_ENCODER_A, INPUT_PULLUP);
  pinMode(BOTTOM_ENCODER_B, INPUT_PULLUP);
  pinMode(TOP_LIMIT_FRONT, INPUT_PULLUP);
  pinMode(TOP_LIMIT_BACK, INPUT_PULLUP);
  pinMode(BOTTOM_LIMIT_FRONT, INPUT_PULLUP);
  pinMode(BOTTOM_LIMIT_BACK, INPUT_PULLUP);
}

bool isPressed(uint32_t pin) {
  return digitalRead(pin) == LOW;
}

void motorWrite(uint32_t pinA, uint32_t pinB, int pwm) {
  pwm = constrain(pwm, -255, 255);
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

void bottomMotor(int pwm) {
  // Directional limit safety is always active.
  if ((pwm < 0 && isPressed(BOTTOM_LIMIT_FRONT)) ||
      (pwm > 0 && isPressed(BOTTOM_LIMIT_BACK))) {
    pwm = 0;
  }
  motorWrite(BOTTOM_MOTOR_A, BOTTOM_MOTOR_B, pwm);
}

void topMotor(int pwm) {
  // Original convention: positive goes FRONT, negative goes BACK.
  if ((pwm > 0 && isPressed(TOP_LIMIT_FRONT)) ||
      (pwm < 0 && isPressed(TOP_LIMIT_BACK))) {
    pwm = 0;
  }
  motorWrite(TOP_MOTOR_A, TOP_MOTOR_B, pwm);
}
