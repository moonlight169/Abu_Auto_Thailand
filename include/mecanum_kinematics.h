#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

class MecanumKinematics {
public:
    MecanumKinematics(float lrDistance, float frDistance, float wheelRadius);

    void inverseKinematics(float vx, float vy, float omega, 
                           float &fl, float &fr, float &rl, float &rr);

private:
    float _l;
    float _w;
    float _radius;
};

#endif