#include <Arduino.h>
#include "pid.h"

PID::PID(float kp, float ki, float kd, int outMin, int outMax){
    this->_kp = kp;
    this->_ki = ki;
    this->_kd = kd;
    this->_outMin = outMin;
    this->_outMax = outMax;
}

int PID::compute(float setpoint, float measured){
    unsigned long now = millis();

    if (this->_firstRun){
        this->_lastTime = now;
        this->_firstRun = false;
    }

    float dt = (now - this->_lastTime) / 1000.0;
    if (dt <= 0){
        dt = 0.001;
    }

    float error = setpoint - measured;

    this->_integral += error * dt;
    this->_integral = constrain(this->_integral, this->_outMin, this->_outMax);

    float derivative = (error - this->_lastError) / dt;

    float output = (this->_kp * error) + (this->_ki * this->_integral) + (this->_kd * derivative);

    this->_lastError = error;
    this->_lastTime = now;

    return constrain((int)output, this->_outMin, this->_outMax);
}

void PID::reset(){
    this->_integral = 0;
    this->_lastError = 0;
    this->_firstRun = true;
}
