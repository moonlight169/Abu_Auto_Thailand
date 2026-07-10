#include <Arduino.h>

#include "config_wheels.h"
#include "motor.h"

Motor MT1(motor1_A, motor1_B);

void setup(){

}

void loop(){
    MT1.smoothRun(100);
    delay(4000);
    MT1.smoothRun(-100);
    delay(4000);
}