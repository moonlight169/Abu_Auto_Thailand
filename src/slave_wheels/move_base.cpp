#include "move_base.h"
#include <Arduino.h>

MoveBase::MoveBase(Wheel& fl, Wheel& fr, Wheel& rl, Wheel& rr, MecanumKinematics& kinematics)
    : _fl(fl), _fr(fr), _rl(rl), _rr(rr), _kinematics(kinematics) {}

void MoveBase::move(float vx, float vy, float omega) {
    vx    = constrain(vx,    -1.000f, 1.000f);
    vy    = constrain(vy,    -1.000f, 1.000f);
    omega = constrain(omega, -1.500f, 1.500f);

    MecanumKinematics::RPM req_rpm = this->_kinematics.getRPM(vx, vy, omega);
    float max_rpm = 500.0f;

    int pwm_fl = constrain((req_rpm.fl / max_rpm) * 255, -255, 255);
    int pwm_fr = constrain((req_rpm.fr / max_rpm) * 255, -255, 255);
    int pwm_rl = constrain((req_rpm.rl / max_rpm) * 255, -255, 255);
    int pwm_rr = constrain((req_rpm.rr / max_rpm) * 255, -255, 255);

    this->_fl.run(pwm_fl);
    this->_fr.run(pwm_fr);
    this->_rl.run(pwm_rl);
    this->_rr.run(pwm_rr);
}

void MoveBase::moveSmooth(float vx, float vy, float omega) {
    vx    = constrain(vx,    -1.000f, 1.000f);
    vy    = constrain(vy,    -1.000f, 1.000f);
    omega = constrain(omega, -1.500f, 1.500f);

    MecanumKinematics::RPM req_rpm = this->_kinematics.getRPM(vx, vy, omega);
    float max_rpm = 500.0f;

    int pwm_fl = constrain((req_rpm.fl / max_rpm) * 255, -255, 255);
    int pwm_fr = constrain((req_rpm.fr / max_rpm) * 255, -255, 255);
    int pwm_rl = constrain((req_rpm.rl / max_rpm) * 255, -255, 255);
    int pwm_rr = constrain((req_rpm.rr / max_rpm) * 255, -255, 255);

    this->_fl.smoothRun(pwm_fl);
    this->_fr.smoothRun(pwm_fr);
    this->_rl.smoothRun(pwm_rl);
    this->_rr.smoothRun(pwm_rr);
}

void MoveBase::update() {
    this->_fl.update();
    this->_fr.update();
    this->_rl.update();
    this->_rr.update();
}

void MoveBase::stop() {
    this->move(0.000f, 0.000f, 0.000f);
}

MoveBase::BaseLog MoveBase::getLog() {
    MoveBase::BaseLog log;

    log.fl.count = this->_fl.getCount();
    log.fl.rpm   = this->_fl.getRPM();
    log.fl.pwm   = this->_fl.getPWM();

    log.fr.count = this->_fr.getCount();
    log.fr.rpm   = this->_fr.getRPM();
    log.fr.pwm   = this->_fr.getPWM();

    log.rl.count = this->_rl.getCount();
    log.rl.rpm   = this->_rl.getRPM();
    log.rl.pwm   = this->_rl.getPWM();

    log.rr.count = this->_rr.getCount();
    log.rr.rpm   = this->_rr.getRPM();
    log.rr.pwm   = this->_rr.getPWM();

    return log;
}

void MoveBase::CountDebug() {
    MoveBase::BaseLog data = this->getLog();
    Serial.print("Count Log -> FL: ");  Serial.print(data.fl.count);
    Serial.print(" | FR: ");            Serial.print(data.fr.count);
    Serial.print(" | RL: ");            Serial.print(data.rl.count);
    Serial.print(" | RR: ");            Serial.println(data.rr.count);
}

void MoveBase::RPMDebug() {
    MoveBase::BaseLog data = this->getLog();
    Serial.print("RPM -> FL: ");  Serial.print(data.fl.rpm, 1);
    Serial.print(" | FR: ");     Serial.print(data.fr.rpm, 1);
    Serial.print(" | RL: ");     Serial.print(data.rl.rpm, 1);
    Serial.print(" | RR: ");     Serial.println(data.rr.rpm, 1);
}

void MoveBase::PWMDebug() {
    MoveBase::BaseLog data = this->getLog();
    Serial.print("PWM Log -> FL: ");  Serial.print(data.fl.pwm, 1);
    Serial.print(" | FR: ");         Serial.print(data.fr.pwm, 1);
    Serial.print(" | RL: ");         Serial.print(data.rl.pwm, 1);
    Serial.print(" | RR: ");         Serial.println(data.rr.pwm, 1);
}