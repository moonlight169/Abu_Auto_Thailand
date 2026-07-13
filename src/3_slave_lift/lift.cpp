#include <Arduino.h>
#include "lift.h"

const int LIFT_HOME_SPEED = 60;

const long LIFT_PULSE_TOLERANCE = 20;

//TODO: ค่าเริ่มต้น ยังไม่ tune กับของจริง แยก front/back เพราะน้ำหนักโหลดสองฝั่งไม่เท่ากัน
const float LIFT_PID_KP_FRONT = 0.08;
const float LIFT_PID_KI_FRONT = 0.02;
const float LIFT_PID_KD_FRONT = 0.01;

const float LIFT_PID_KP_BACK = 0.08;
const float LIFT_PID_KI_BACK = 0.02;
const float LIFT_PID_KD_BACK = 0.01;

const int LIFT_PID_OUT_MIN = -255;
const int LIFT_PID_OUT_MAX = 255;

Lift::Lift(int frontMotorA, int frontMotorB, int backMotorA, int backMotorB,
           int encFrontA, int encFrontB, int encBackA, int encBackB,
           int swFrontTop, int swFrontBottom, int swBackTop, int swBackBottom)
    : _motorFront(frontMotorA, frontMotorB),
      _motorBack(backMotorA, backMotorB),
      _encoderFront(encFrontA, encFrontB, 1.0),
      _encoderBack(encBackA, encBackB, 1.0),
      _pidFront(LIFT_PID_KP_FRONT, LIFT_PID_KI_FRONT, LIFT_PID_KD_FRONT, LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX),
      _pidBack(LIFT_PID_KP_BACK, LIFT_PID_KI_BACK, LIFT_PID_KD_BACK, LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX){

    this->_swFrontTop = swFrontTop;
    this->_swFrontBottom = swFrontBottom;
    this->_swBackTop = swBackTop;
    this->_swBackBottom = swBackBottom;

    pinMode(swFrontTop, INPUT_PULLUP);
    pinMode(swFrontBottom, INPUT_PULLUP);
    pinMode(swBackTop, INPUT_PULLUP);
    pinMode(swBackBottom, INPUT_PULLUP);
}

void Lift::setZero(){
    this->_state = LIFT_HOMING;
    this->_frontHomed = false;
    this->_backHomed = false;
}

void Lift::liftTo(long targetPulse){
    this->_targetPulse = targetPulse;
    this->_state = LIFT_MOVING;
}

void Lift::stop(){
    this->_state = LIFT_IDLE;
    this->_motorFront.run(0);
    this->_motorBack.run(0);
}

void Lift::applySafetyCutoff(){
    bool frontTop = (digitalRead(this->_swFrontTop) == LOW);
    bool frontBottom = (digitalRead(this->_swFrontBottom) == LOW);
    bool backTop = (digitalRead(this->_swBackTop) == LOW);
    bool backBottom = (digitalRead(this->_swBackBottom) == LOW);

    if (frontTop && this->_motorFront.getSpeed() < 0){
        this->_motorFront.run(0);
    }
    if (frontBottom && this->_motorFront.getSpeed() > 0){
        this->_motorFront.run(0);
    }
    if (backTop && this->_motorBack.getSpeed() < 0){
        this->_motorBack.run(0);
    }
    if (backBottom && this->_motorBack.getSpeed() > 0){
        this->_motorBack.run(0);
    }
}

void Lift::update(){
    this->applySafetyCutoff();

    if (this->_state == LIFT_HOMING){
        bool frontBottom = (digitalRead(this->_swFrontBottom) == LOW);
        bool backBottom = (digitalRead(this->_swBackBottom) == LOW);

        if (!frontBottom){
            this->_motorFront.run(LIFT_HOME_SPEED);
        } else {
            this->_motorFront.run(0);
            if (!this->_frontHomed){
                this->_encoderFront.reset();
                this->_frontHomed = true;
            }
        }

        if (!backBottom){
            this->_motorBack.run(LIFT_HOME_SPEED);
        } else {
            this->_motorBack.run(0);
            if (!this->_backHomed){
                this->_encoderBack.reset();
                this->_backHomed = true;
            }
        }

        if (this->_frontHomed && this->_backHomed){
            this->_state = LIFT_IDLE;
        }
    }
    else if (this->_state == LIFT_MOVING){
        int frontOutput = this->_pidFront.compute(this->_targetPulse, this->_encoderFront.getCount());
        this->_motorFront.run(frontOutput);

        int backOutput = this->_pidBack.compute(this->_targetPulse, this->_encoderBack.getCount());
        this->_motorBack.run(backOutput);
    }
}

void Lift::handleFrontA(){
    this->_encoderFront.handleA();
}

void Lift::handleFrontB(){
    this->_encoderFront.handleB();
}

void Lift::handleBackA(){
    this->_encoderBack.handleA();
}

void Lift::handleBackB(){
    this->_encoderBack.handleB();
}

long Lift::getFrontCount(){
    return this->_encoderFront.getCount();
}

long Lift::getBackCount(){
    return this->_encoderBack.getCount();
}

void Lift::CountDebug(){
    Serial.print("Lift Count -> Front: ");
    Serial.print(this->getFrontCount());
    Serial.print(" | Back: ");
    Serial.println(this->getBackCount());
}

bool Lift::isBusy(){
    return this->_state != LIFT_IDLE;
}

bool Lift::atTarget(){
    long frontError = abs(this->_targetPulse - this->_encoderFront.getCount());
    long backError = abs(this->_targetPulse - this->_encoderBack.getCount());

    return (frontError <= LIFT_PULSE_TOLERANCE) && (backError <= LIFT_PULSE_TOLERANCE);
}
