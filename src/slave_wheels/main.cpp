#include <Arduino.h>

#include "config_wheels.h"
#include "motor.h"
#include "encoder.h"
#include "wheel.h"

Wheel wheel1(motor1_A, motor1_B, enc_A, enc_B, PULSE_PER_REV);

void isrFL_A() { wheel1.handleA(); }
void isrFL_B() { wheel1.handleB(); }


void setup(){
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(enc_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_B), isrFL_B, CHANGE);
}

void loop(){
    wheel1.smoothRun(0);

    Serial.print("Count: "); Serial.print(wheel1.getCount());
    Serial.print(" | RPM: "); Serial.println(wheel1.getRPM(), 1);

    delay(100);
}