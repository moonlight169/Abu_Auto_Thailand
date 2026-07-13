#ifndef PID_H
#define PID_H

class PID{
public:
    PID(float kp, float ki, float kd, int outMin, int outMax);

    int compute(float setpoint, float measured);
    void reset();

private:
    float _kp;
    float _ki;
    float _kd;
    int _outMin;
    int _outMax;

    float _integral = 0;
    float _lastError = 0;
    unsigned long _lastTime = 0;
    bool _firstRun = true;
};

#endif
