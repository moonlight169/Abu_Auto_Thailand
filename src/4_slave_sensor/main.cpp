#include <Arduino.h>
#include <Servo.h>

#include "config/config_sensor.h"
#include "protocol.h"

Servo armServo;
Servo spinServo;

SensorLinkReceiver sensorReceiver;

unsigned long lastLogTime = 0;

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    armServo.attach(arm_servo_pin);
    spinServo.attach(spin_servo_pin);

    pinMode(Relay_1, OUTPUT);
    pinMode(Relay_2, OUTPUT);
    pinMode(Relay_3, OUTPUT);
    pinMode(Relay_4, OUTPUT);

    digitalWrite(Relay_1, HIGH);
    digitalWrite(Relay_2, HIGH);
    digitalWrite(Relay_3, HIGH);
    digitalWrite(Relay_4, HIGH);

    armServo.write(0);
    spinServo.write(5);
}

void loop(){
    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();
        sensorLinkReceiverFeed(sensorReceiver, incomingByte);
    }

    if (sensorReceiver.hasNewServoCommand){
        armServo.write(sensorReceiver.lastServoCommand.armAngle);
        spinServo.write(sensorReceiver.lastServoCommand.spinAngle);
        sensorReceiver.hasNewServoCommand = false;
    }

    if (sensorReceiver.hasNewRelayCommand){
        uint8_t state = sensorReceiver.lastRelayCommand.relayState;
        digitalWrite(Relay_1, (state & 0x01) ? LOW : HIGH);
        digitalWrite(Relay_2, (state & 0x02) ? LOW : HIGH);
        digitalWrite(Relay_3, (state & 0x04) ? LOW : HIGH);
        digitalWrite(Relay_4, (state & 0x08) ? LOW : HIGH);
        sensorReceiver.hasNewRelayCommand = false;
    }
    unsigned long now = millis();
    if (now - lastLogTime >= 1000) {
        Serial.println(sensorReceiver.lastServoCommand.armAngle);
        Serial.println(sensorReceiver.lastServoCommand.spinAngle);

        uint8_t relayState = sensorReceiver.lastRelayCommand.relayState;
        Serial.print("Relay1: ");  Serial.print((relayState & 0x01) ? "ON" : "OFF");
        Serial.print(" | Relay2: "); Serial.print((relayState & 0x02) ? "ON" : "OFF");
        Serial.print(" | Relay3: "); Serial.print((relayState & 0x04) ? "ON" : "OFF");
        Serial.print(" | Relay4: "); Serial.println((relayState & 0x08) ? "ON" : "OFF");
        lastLogTime = now;
    }
}