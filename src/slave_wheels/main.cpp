#include <Arduino.h>

#include "config_wheels.h"
#include "wheel.h"
#include "mecanum_kinematics.h"
#include "move_base.h"

Wheel wheelFL(MotorFL_A, MotorFL_B, EncFL_A, EncFL_B, PULSE_PER_REV);
Wheel wheelFR(MotorFR_A, MotorFR_B, EncFR_A, EncFR_B, PULSE_PER_REV);
Wheel wheelRL(MotorRL_A, MotorRL_B, EncRL_A, EncRL_B, PULSE_PER_REV);
Wheel wheelRR(MotorRR_A, MotorRR_B, EncRR_A, EncRR_B, PULSE_PER_REV);

MecanumKinematics kinematics(LR_WHEELS_DISTANCE, FR_WHEELS_DISTANCE, WHEEL_RADIUS);

MoveBase robotDrive(wheelFL, wheelFR, wheelRL, wheelRR, kinematics);

unsigned long lastLogTime = 0;

void isrFL_A() { wheelFL.handleA(); }
void isrFL_B() { wheelFL.handleB(); }

void isrFR_A() { wheelFR.handleA(); }
void isrFR_B() { wheelFR.handleB(); }

void isrRL_A() { wheelRL.handleA(); }
void isrRL_B() { wheelRL.handleB(); }

void isrRR_A() { wheelRR.handleA(); }
void isrRR_B() { wheelRR.handleB(); }

void setup(){
    Serial.begin(115200);
    robotDrive.stop();

    attachInterrupt(digitalPinToInterrupt(EncFL_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFL_B), isrFL_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncFR_A), isrFR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFR_B), isrFR_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncRL_A), isrRL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncRL_B), isrRL_B, CHANGE);

    attachInterrupt(digitalPinToInterrupt(EncRR_A), isrRR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncRR_B), isrRR_B, CHANGE);
    
    delay(2000);
}

void loop() {
    robotDrive.move(0.000f, 0.000f, 0.000f);

    unsigned long now = millis();
    if (now - lastLogTime >= 100) { 
        robotDrive.RPMDebug();
        robotDrive.PWMDebug();
        lastLogTime = now;
    }
}