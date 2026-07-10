#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

class MecanumKinematics {
public:
    struct RPM {
        float fl;
        float fr;
        float rl;
        float rr;
    };

    MecanumKinematics(float lrDistance, float frDistance, float wheelRadius);

    RPM getRPM(float vx, float vy, float omega);

private:
    float _l;
    float _w;
    float _radius;
};

#endif