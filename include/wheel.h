#ifndef WHEEL_H
#define WHEEL_H

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"

class Wheel{
public:
    Wheel(int motorA, int motorB, int encA, int encB, float ppr);

    void run(int speed);
    void smoothRun(int targetSpeed);
    void handleA();
    void handleB();
    float getRPM();
    float getDistance(float ppm);
    long getCount();

private:
    Motor _motor;
    Encoder _encoder;
};

#endif