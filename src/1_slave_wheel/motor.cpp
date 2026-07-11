#include <Arduino.h>
#include "motor.h"
#include "config_wheels.h"

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

void Motor::smoothRun(int targetSpeed) {
    this->_targetSpeed = constrain(targetSpeed, -255, 255);
}

void Motor::update() {
    unsigned long now = millis();

    if (this->_speed < this->_targetSpeed) {
        if (now - this->_lastStepMillis >= stepDelay) {
            this->_speed++;
            this->run();
            this->_lastStepMillis = now;
        }
    } else if (this->_speed > this->_targetSpeed) {
        if (now - this->_lastStepMillis >= stepDelay) {
            this->_speed--;
            this->run();
            this->_lastStepMillis = now;
        }
    }
}