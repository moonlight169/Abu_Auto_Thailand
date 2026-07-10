#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int pinA, int pinB, float ppr);
    void handleA();
    void handleB();
    float getRPM();
    float getDistance(float ppm);
    long getCount();

private:
    int _pinA;
    int _pinB;
    float _ppr;
    float _rpm;

    volatile long _count = 0;
    unsigned long _lastTime = 0;
    long _lastCount = 0;
};

#endif