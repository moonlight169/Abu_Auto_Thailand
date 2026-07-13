#include <Arduino.h>
#include "encoder.h"

Encoder::Encoder(int pinA, int pinB, float ppr){
    this->_pinA = pinA;
    this->_pinB = pinB;
    this->_ppr = ppr;

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);

    this->_lastTime = millis();
}

void Encoder::handleA(){
    if (digitalRead(this->_pinA) != digitalRead(this->_pinB)) {
        this->_count++;
    } else {
        this->_count--;
    }
}

void Encoder::handleB(){
    if (digitalRead(this->_pinA) == digitalRead(this->_pinB)) {
        this->_count++;
    } else {
        this->_count--;
    }
}

void Encoder::reset(){
    noInterrupts();
    this->_count = 0;
    interrupts();
}

float Encoder::getRPM(){
    unsigned long now = millis();
    unsigned long dt = now - this->_lastTime;

    if (dt >= 20) {
        float dtMin = dt / 60000.0;

        noInterrupts();
        long currentCount = this->_count;
        interrupts();

        this->_rpm = ((currentCount - this->_lastCount) / this->_ppr) / dtMin;
        this->_lastTime = now;
        this->_lastCount = currentCount;
    }
    return this->_rpm;
}

float Encoder::getDistance(float ppm){
    noInterrupts();
    long c = this->_count;
    interrupts();
    return c / ppm;
}

long Encoder::getCount(){
    noInterrupts();
    long c = this->_count;
    interrupts();
    return c;
}