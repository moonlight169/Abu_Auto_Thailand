#include "Arduino.h"
#include "PID.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd)
    : min_val_(min_val),
      max_val_(max_val),
      kp_(kp),
      ki_(ki),
      kd_(kd),
      integral_(0),
      derivative_(0),
      prev_error_(0)
{
}

double PID::compute(float setpoint, float measured_value)
{
    double error = setpoint - measured_value;
    integral_ += error;

    // 🔒 Anti-Windup: จำกัดการสะสมค่า I
    if (integral_ > integral_max_) integral_ = integral_max_;
    if (integral_ < integral_min_) integral_ = integral_min_;

    derivative_ = error - prev_error_;
    prev_error_ = error;

    if (setpoint == 0)
    {
        integral_ = 0;
        derivative_ = 0;
    }

    double pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
    return constrain(pid, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    integral_ = 0;
    derivative_ = 0;
    prev_error_ = 0;
}

void PID::limitIntegral(float i_min, float i_max)
{
    integral_min_ = i_min;
    integral_max_ = i_max;
}
