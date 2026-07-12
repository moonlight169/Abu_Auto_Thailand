#include <Arduino.h>
#include <Servo.h>

#include "config_sensor.h"
#include "protocol.h"

Servo armServo;
Servo spinServo;

ServoReceiver servoReceiver;

unsigned long lastLogTime = 0;

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    armServo.attach(arm_servo_pin);
    spinServo.attach(spin_servo_pin);
}

void loop(){
    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();
        servoReceiverFeed(servoReceiver, incomingByte);
    }

    if (servoReceiver.hasNewCommand){
        armServo.write(servoReceiver.lastCommand.armAngle);
        spinServo.write(servoReceiver.lastCommand.spinAngle);
        servoReceiver.hasNewCommand = false;
    }

    unsigned long now = millis();
    if (now - lastLogTime >= 100) {
        Serial.print("ServoArm_degree: ");
        Serial.println(servoReceiver.lastCommand.armAngle);
        Serial.print("ServoSpin_degree: ");
        Serial.println(servoReceiver.lastCommand.spinAngle);
        lastLogTime = now;
    }
}