#include <Arduino.h>
#include "motor.h"

Motor::Motor(int pinA, int pinB){
    this->_pinA = pinA;
    this->_pinB = pinB;

    analogWriteResolution(8);
    analogWriteFrequency(5000);
    
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);

    this->_targetSpeed = 0;
    this->_lastStepMillis = 0;
}

Motor::~Motor(){
}

void Motor::setspeed(int speed){
    this->_speed = constrain(speed, -255, 255);
}

int Motor::getSpeed(){
    return this->_speed;
}

void Motor::run(int speed){
    this->setspeed(speed);
    this->run();
}

void Motor::run(){
    if (this->_speed > 0){
        analogWrite(this->_pinA, abs(this->_speed));
        analogWrite(this->_pinB, 0);
    } else if (this->_speed < 0){
        analogWrite(this->_pinA, 0);
        analogWrite(this->_pinB, abs(this->_speed)); 
    } else {
        analogWrite(this->_pinB, 0);
        analogWrite(this->_pinA, 0);
    }
}

void Motor::lock() {
    this->_speed = 0;
    this->_targetSpeed = 0;  
    analogWrite(this->_pinA, 255);
    analogWrite(this->_pinB, 255); 
}