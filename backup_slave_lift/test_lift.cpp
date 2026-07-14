#include <Arduino.h>
#include "config_lift.h"
#include "lift.h"

#define LOOP_HZ 50                        

Motor liftFront(lift_frontA, lift_frontB);
Motor liftBack(lift_backA, lift_backB);


unsigned long lastLogTime = 0;
bool testLiftSent = false;

void setup(){
    Serial.begin(115200);
}

void loop() {
    bool lmup = (digitalRead(L_SW_lift_fronttop_1) == LOW);
    bool lmdown = (digitalRead(L_SW_lift_frontbottom_3) == LOW);

    if (!lmdown) {
        liftFront.run(60);
    } else {
        liftFront.run(0);
    }
}