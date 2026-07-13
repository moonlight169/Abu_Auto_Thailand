#include <Arduino.h>
#include "config_arm.h"
#include "motor.h"

unsigned long lastLogTime = 0;
Motor armDown(arm_downA, arm_downB);
Motor armUp(arm_upA, arm_upB);

void setup(){
    Serial.begin(115200);

    pinMode(L_SW_arm_up_front_4, INPUT_PULLUP);
    pinMode(L_SW_arm_up_back_2, INPUT_PULLUP);
    pinMode(L_SW_arm_down_front_8, INPUT_PULLUP);
    pinMode(L_SW_arm_down_back_9, INPUT_PULLUP);
}

void loop() {
    //หมุนไปหน้า
    // if (!down_front) armDown.run(150); else armDown.run(0);
    
    //หมุนไปหลัง
    // if (!down_back) armDown.run(-40); else armDown.run(0);

    bool up_front = (digitalRead(L_SW_arm_up_front_4) == LOW);
    bool up_back = (digitalRead(L_SW_arm_up_back_2) == LOW);
    bool down_front = (digitalRead(L_SW_arm_down_front_8) == LOW);
    bool down_back = (digitalRead(L_SW_arm_down_back_9) == LOW);
}
