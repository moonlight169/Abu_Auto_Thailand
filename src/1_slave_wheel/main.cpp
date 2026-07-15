#include <Arduino.h>

#include "config/config_wheels.h"
#include "wheel.h"
#include "protocol.h"
#include "Kinematics.h"
#include "PID.h"

extern Wheel wheelFL;
extern Wheel wheelFR;
extern Wheel wheelRL;
extern Wheel wheelRR;

WheelReceiver wheelReceiver;

Kinematics kinematics(Kinematics::MECANUM, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

PID MotorFL_Pid(PWM_MIN, PWM_MAX, FL_K_P, FL_K_I, FL_K_D);
PID MotorFR_Pid(PWM_MIN, PWM_MAX, FR_K_P, FR_K_I, FR_K_D);
PID MotorRL_Pid(PWM_MIN, PWM_MAX, RL_K_P, RL_K_I, RL_K_D);
PID MotorRR_Pid(PWM_MIN, PWM_MAX, RR_K_P, RR_K_I, RR_K_D);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

int current_rpm1, current_rpm2, current_rpm3, current_rpm4;

unsigned long prev_control_time = 0;
unsigned long g_prev_command_time = 0;

#define COMMAND_RATE 80          // Hz
#define WHEEL_CMD_TIMEOUT_MS 300 // ms

void isrFL_A() { wheelFL.handleA(); }
void isrFL_B() { wheelFL.handleB(); }

void isrFR_A() { wheelFR.handleA(); }
void isrFR_B() { wheelFR.handleB(); }

void isrRL_A() { wheelRL.handleA(); }
void isrRL_B() { wheelRL.handleB(); }

void isrRR_A() { wheelRR.handleA(); }
void isrRR_B() { wheelRR.handleB(); }

void moveBase();

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    wheelFL.run(0);
    wheelFR.run(0);
    wheelRL.run(0);
    wheelRR.run(0);

    attachInterrupt(digitalPinToInterrupt(EncFL_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFL_B), isrFL_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncFR_A), isrFR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFR_B), isrFR_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncRL_A), isrRL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncRL_B), isrRL_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncRR_A), isrRR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncRR_B), isrRR_B, CHANGE);
}

void loop() {
    current_rpm1 = wheelFL.getRPM();
    current_rpm2 = wheelFR.getRPM();
    current_rpm3 = wheelRL.getRPM();
    current_rpm4 = wheelRR.getRPM();

    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();
        wheelReceiverFeed(wheelReceiver, incomingByte);
    }

    if (wheelReceiver.hasNewCommand){
        g_req_linear_vel_x = wheelReceiver.lastCommand.vx;
        g_req_linear_vel_y = wheelReceiver.lastCommand.vy;
        g_req_angular_vel_z = wheelReceiver.lastCommand.omega;
        g_prev_command_time = millis();
        wheelReceiver.hasNewCommand = false;
    }

    if (millis() - g_prev_command_time > WHEEL_CMD_TIMEOUT_MS){
        g_req_linear_vel_x = 0;
        g_req_linear_vel_y = 0;
        g_req_angular_vel_z = 0;
    }

    unsigned long now = millis();
    if ((now - prev_control_time) >= (1000 / COMMAND_RATE)){
        moveBase();
        prev_control_time = now;
    }
}

void moveBase()
{
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    wheelFL.run(MotorFL_Pid.compute(req_rpm.motor1, current_rpm1));
    wheelFR.run(MotorFR_Pid.compute(req_rpm.motor2, current_rpm2));
    wheelRL.run(MotorRL_Pid.compute(req_rpm.motor3, current_rpm3));
    wheelRR.run(MotorRR_Pid.compute(req_rpm.motor4, current_rpm4));
}