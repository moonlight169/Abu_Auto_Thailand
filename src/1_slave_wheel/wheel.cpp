#include <Arduino.h>
#include "wheel.h"
#include "config/config_wheels.h"

// ========== Global wheel objects ==========
Wheel wheelFL(MotorFL_A, MotorFL_B, EncFL_A, EncFL_B, PULSE_PER_REV);
Wheel wheelFR(MotorFR_A, MotorFR_B, EncFR_A, EncFR_B, PULSE_PER_REV);
Wheel wheelRL(MotorRL_A, MotorRL_B, EncRL_A, EncRL_B, PULSE_PER_REV);
Wheel wheelRR(MotorRR_A, MotorRR_B, EncRR_A, EncRR_B, PULSE_PER_REV);

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

float Wheel::getDistance(float ppm){
    return this->_encoder.getDistance(ppm);
}

long Wheel::getCount(){
    return this->_encoder.getCount();
}