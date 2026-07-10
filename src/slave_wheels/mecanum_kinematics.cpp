#include "mecanum_kinematics.h"

MecanumKinematics::MecanumKinematics(float lrDistance, float frDistance, float wheelRadius) {
    this->_l = lrDistance / 2.0f; 
    this->_w = frDistance / 2.0f;
    this->_radius = wheelRadius;
}

void MecanumKinematics::inverseKinematics(float vx, float vy, float omega, 
                                       float &fl, float &fr, float &rl, float &rr) {
    float fl_linear = vx - vy - (this->_l + this->_w) * omega;
    float fr_linear = vx + vy + (this->_l + this->_w) * omega;
    float rl_linear = vx + vy - (this->_l + this->_w) * omega;
    float rr_linear = vx - vy + (this->_l + this->_w) * omega;

    fl = fl_linear / this->_radius;
    fr = fr_linear / this->_radius;
    rl = rl_linear / this->_radius;
    rr = rr_linear / this->_radius;
}