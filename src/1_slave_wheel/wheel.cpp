#include <Arduino.h>
#include "wheel.h"

Wheel::Wheel(int motorA, int motorB, int encA, int encB, float ppr)
    : _motor(motorA, motorB), _encoder(encA, encB, ppr) {}

void Wheel::run(int speed){
    this->_motor.run(speed);
}

void Wheel::smoothRun(int targetSpeed){
    this->_motor.smoothRun(targetSpeed);
}

void Wheel::update(){
    this->_motor.update();
}

void Wheel::handleA(){
    this->_encoder.handleA();
}

void Wheel::handleB(){
    this->_encoder.handleB();
}

float Wheel::getRPM(){
    return this->_encoder.getRPM();
}

int Wheel::getPWM(){
    return this->_motor.getSpeed();
}

long Wheel::getCount(){
    return this->_encoder.getCount();
}