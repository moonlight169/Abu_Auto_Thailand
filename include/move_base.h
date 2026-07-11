#ifndef MOVE_BASE_H
#define MOVE_BASE_H

#include "wheel.h"
#include "mecanum_kinematics.h"

class MoveBase {
public:
    struct WheelLog {
        long count;
        float rpm;
        float pwm;
    };
    struct BaseLog {
        WheelLog fl;
        WheelLog fr;
        WheelLog rl;
        WheelLog rr;
    };

    MoveBase(Wheel& fl, Wheel& fr, Wheel& rl, Wheel& rr, MecanumKinematics& kinematics);
    void move(float vx, float vy, float omega);
    void moveSmooth(float vx, float vy, float omega);
    void update();
    void stop();

    BaseLog getLog();
    
    void RPMDebug();
    void PWMDebug();

private:
    Wheel& _fl;
    Wheel& _fr;
    Wheel& _rl;
    Wheel& _rr;
    MecanumKinematics& _kinematics;
};

#endif