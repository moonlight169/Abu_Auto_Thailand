#include <Arduino.h>
#include "mecanum_kinematics.h"

MecanumKinematics::MecanumKinematics(float lrDistance, float frDistance, float wheelRadius) {
    this->_l = lrDistance / 2.0f;
    this->_w = frDistance / 2.0f;
    this->_radius = wheelRadius;
}

MecanumKinematics::RPM MecanumKinematics::getRPM(float vx, float vy, float omega) {
    float fl_v = vx - vy - (this->_l + this->_w) * omega;
    float fr_v = vx + vy + (this->_l + this->_w) * omega;
    float rl_v = vx + vy - (this->_l + this->_w) * omega;
    float rr_v = vx - vy + (this->_l + this->_w) * omega;

    float to_rpm = 60.0f / (2.0f * PI * this->_radius);

    MecanumKinematics::RPM req_rpm;
    req_rpm.fl = fl_v * to_rpm;
    req_rpm.fr = fr_v * to_rpm;
    req_rpm.rl = rl_v * to_rpm;
    req_rpm.rr = rr_v * to_rpm;

    return req_rpm;
}