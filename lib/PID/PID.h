#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
public:
    PID(float min_val, float max_val, float kp, float ki, float kd);
    double compute(float setpoint, float measured_value);
    void updateConstants(float kp, float ki, float kd);
    void limitIntegral(float i_min, float i_max); // ✅ Anti-Windup

private:
    float min_val_;
    float max_val_;
    float kp_;
    float ki_;
    float kd_;
    double integral_;
    double derivative_;
    double prev_error_;

    float integral_min_ = -1000;  // ค่าเริ่มต้น (อิสระ)
    float integral_max_ = 1000;
};

#endif
