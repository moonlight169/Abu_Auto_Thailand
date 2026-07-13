#ifndef LIFT_H
#define LIFT_H

#include <Arduino.h>
#include "motor.h"
#include "encoder.h"
#include "pid.h"

enum LiftState {
    LIFT_IDLE,
    LIFT_HOMING,
    LIFT_MOVING
};

class Lift{
public:
    Lift(int frontMotorA, int frontMotorB, int backMotorA, int backMotorB,
         int encFrontA, int encFrontB, int encBackA, int encBackB,
         int swFrontTop, int swFrontBottom, int swBackTop, int swBackBottom);

    void setZero();
    void liftTo(long targetPulse);
    void stop();
    void update();

    void handleFrontA();
    void handleFrontB();
    void handleBackA();
    void handleBackB();

    long getFrontCount();
    long getBackCount();
    bool isBusy();
    bool atTarget();
    void CountDebug();

private:
    Motor _motorFront;
    Motor _motorBack;
    Encoder _encoderFront;
    Encoder _encoderBack;
    PID _pidFront;
    PID _pidBack;

    int _swFrontTop;
    int _swFrontBottom;
    int _swBackTop;
    int _swBackBottom;

    LiftState _state = LIFT_IDLE;
    long _targetPulse = 0;

    bool _frontHomed = false;
    bool _backHomed = false;

    void applySafetyCutoff();
};

#endif
