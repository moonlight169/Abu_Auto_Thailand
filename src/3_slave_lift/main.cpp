#include <Arduino.h>
#include "config_lift.h"
#include "lift.h"

Lift lift(lift_frontA, lift_frontB, lift_backA, lift_backB,
          enc_lift_frontA, enc_lift_frontB, enc_lift_backA, enc_lift_backB,
          L_SW_lift_fronttop_1, L_SW_lift_frontbottom_3, L_SW_lift_backtop_5, L_SW_lift_backbottom_7);

void isrEncFrontA() { lift.handleFrontA(); }
void isrEncFrontB() { lift.handleFrontB(); }
void isrEncBackA()  { lift.handleBackA(); }
void isrEncBackB()  { lift.handleBackB(); }

void setup(){
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(enc_lift_frontA), isrEncFrontA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_frontB), isrEncFrontB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_backA), isrEncBackA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_backB), isrEncBackB, CHANGE);

    lift.setZero();
}

void loop() {
    lift.update();
}
