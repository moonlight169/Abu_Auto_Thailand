#include <Arduino.h>
#include "motor.h"

Motor::Motor(int pinA, int pinB){
    this->_pinA = pinA;
    this->_pinB = pinB;

    analogWriteResolution(8);
    analogWriteFrequency(5000);
    
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);

    this->_speed = 0;
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

void Motor::smoothRun(int targetSpeed){
    // Simple acceleration ramp: move speed toward target by max step of 10 per call
    if (targetSpeed > this->_speed){
        this->_speed = min(this->_speed + 10, targetSpeed);
    } else if (targetSpeed < this->_speed){
        this->_speed = max(this->_speed - 10, targetSpeed);
    }
    this->run();
}

void Motor::update(){
    // Gradually move current speed toward target speed
    if (this->_targetSpeed > this->_speed){
        this->_speed = min(this->_speed + 10, this->_targetSpeed);
    } else if (this->_targetSpeed < this->_speed){
        this->_speed = max(this->_speed - 10, this->_targetSpeed);
    }
    this->run();
}

void Motor::lock() {
    this->_speed = 0;
    this->_targetSpeed = 0;  
    analogWrite(this->_pinA, 255);
    analogWrite(this->_pinB, 255); 
}