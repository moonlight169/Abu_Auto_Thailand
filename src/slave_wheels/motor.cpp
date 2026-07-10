#include <Arduino.h>
#include "motor.h"

Motor::Motor(int pinA, int pinB, int maxRPM){
    this->_pinA = pinA;
    this->_pinB = pinB;
    this->_maxRPM = maxRPM;

    analogWriteResolution(8);
    analogWriteFrequency(5000);
    
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
}

Motor::~Motor(){
}

void Motor::setspeed(int speed){
    if (speed > 255) {
        this->_speed = 255;
    } else if (speed < -255){
        this->_speed = -255;
    } else {
        this->_speed = speed;
    }
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

void Motor::smoothRun(int targetSpeed, int stepDelay) {
    int currentSpeed = this->getSpeed();
    
    if (currentSpeed < targetSpeed) {
        while (currentSpeed < targetSpeed) {
            currentSpeed++;
            this->run(currentSpeed);
            delay(stepDelay);
        }
    } else if (currentSpeed > targetSpeed) {
        while (currentSpeed > targetSpeed) {
            currentSpeed--;
            this->run(currentSpeed);
            delay(stepDelay);
        }
    }
}