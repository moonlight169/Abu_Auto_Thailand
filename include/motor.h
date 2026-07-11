#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
public:
    Motor(int pinA, int pinB);
    void setspeed(int speed);
    int getSpeed();
    void run(int speed);
    void run();
    void smoothRun(int targetSpeed);
    void update();
    ~Motor();

private:
    int _pinA;
    int _pinB;
    int _speed = 0;
    int _maxRPM = 0;
    int _targetSpeed = 0;
    unsigned long _lastStepMillis = 0;

};


#endif