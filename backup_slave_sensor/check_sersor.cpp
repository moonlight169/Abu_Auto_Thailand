#include <Arduino.h>
#include <Servo.h>

#include "config_sensor.h"

Servo arm_servo;
Servo spin_servo;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
    }

    pinMode(L_SW_lift_fonttop_1, INPUT_PULLUP);
    pinMode(L_SW_armback_2, INPUT_PULLUP);
    pinMode(L_SW_lift_fontbottom_3, INPUT_PULLUP);
    pinMode(L_SW_armfont_4, INPUT_PULLUP);
    pinMode(L_SW_lift_backtop_5, INPUT_PULLUP);
    pinMode(L_SW_fontrobot_6, INPUT_PULLUP);
    pinMode(L_SW_lift_backbottom_7, INPUT_PULLUP);

    pinMode(Relay_1, OUTPUT);
    pinMode(Relay_2, OUTPUT);
    pinMode(Relay_3, OUTPUT);
    pinMode(Relay_4, OUTPUT);

    digitalWrite(Relay_1, 1);
    digitalWrite(Relay_2, 1);
    digitalWrite(Relay_3, 1);
    digitalWrite(Relay_4, 1);

    arm_servo.attach(arm_servo_pin, 500, 2500);
    spin_servo.attach(spin_servo_pin, 500, 2500); 

    arm_servo.write(0);
    spin_servo.write(5);
}

void loop() {
    Serial.print(" || Limits -> ");
    Serial.print("1: "); Serial.print(digitalRead(L_SW_lift_fonttop_1));
    Serial.print(" | 2: "); Serial.print(digitalRead(L_SW_armback_2));
    Serial.print(" | 3: "); Serial.print(digitalRead(L_SW_lift_fontbottom_3));
    Serial.print(" | 4: "); Serial.print(digitalRead(L_SW_armfont_4));
    Serial.print(" | 5: "); Serial.print(digitalRead(L_SW_lift_backtop_5));
    Serial.print(" | 6: "); Serial.print(digitalRead(L_SW_fontrobot_6));
    Serial.print(" | 7: "); Serial.println(digitalRead(L_SW_lift_backbottom_7));

    delay(200);
}