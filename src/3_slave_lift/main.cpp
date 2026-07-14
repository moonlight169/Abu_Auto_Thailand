#include <Arduino.h>
#include "config_lift.h"
#include "lift.h"

#define LOOP_HZ 50                        

Lift liftAll(lift_frontA, lift_frontB, lift_backA, lift_backB,
          enc_lift_frontA, enc_lift_frontB, enc_lift_backA, enc_lift_backB,
          L_SW_lift_fronttop_1, L_SW_lift_frontbottom_3, L_SW_lift_backtop_5, L_SW_lift_backbottom_7);

void isrEncFrontA() { liftAll.handleFrontA(); }
void isrEncFrontB() { liftAll.handleFrontB(); }
void isrEncBackA()  { liftAll.handleBackA(); }
void isrEncBackB()  { liftAll.handleBackB(); }

unsigned long lastLogTime = 0;
bool testLiftSent = false;

void setup(){
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(enc_lift_frontA), isrEncFrontA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_frontB), isrEncFrontB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_backA), isrEncBackA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_backB), isrEncBackB, CHANGE);

    liftAll.setZero();
}

void loop() {
    liftAll.update();

    if (millis() - lastLogTime >= 1000 / LOOP_HZ){
        if (!liftAll.isBusy() && !testLiftSent){
            liftAll.liftTo(2000, 2000);
            testLiftSent = true;
        }
        liftAll.CountDebug();
        liftAll.PWMDebug();
        lastLogTime = millis();
    }
}