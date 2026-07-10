#include <Arduino.h>

#include "config_wheels.h"
#include "motor.h"
#include "encoder.h"

Motor MT1(motor1_A, motor1_B);
Encoder Encoder1(enc_A, enc_B, PULSE_PER_REV);

void isrFL_A() { Encoder1.handleA(); }
void isrFL_B() { Encoder1.handleB(); }


void setup(){
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(enc_A), isrFL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_B), isrFL_B, CHANGE);
}

void loop(){
    MT1.smoothRun(100);
    float rpm = Encoder1.getRPM();
    long count = Encoder1.getCount();

    Serial.print("Count: "); Serial.print(count);
    Serial.print(" | RPM: "); Serial.println(rpm, 1);

    delay(100);
}