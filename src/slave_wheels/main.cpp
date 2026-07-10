#include <Arduino.h>

#include "config_wheels.h"
#include "motor.h"
#include "encoder.h"
#include "wheel.h"

Wheel wheelFL(MotorFL_A, MotorFL_B, EncFL_A, EncFL_B, PULSE_PER_REV);

void isrFL_A() { wheelFL.handleA(); }
void isrFL_B() { wheelFL.handleB(); }


void setup(){
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(EncFL_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(EncFL_B), isrFL_B, CHANGE);
}

void loop(){
    wheelFL.smoothRun(0);

    Serial.print("Count: "); Serial.print(wheelFL.getCount());
    Serial.print(" | RPM: "); Serial.println(wheelFL.getRPM(), 1);

    delay(100);
}