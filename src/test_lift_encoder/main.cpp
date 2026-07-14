#include <Arduino.h>
#include "config_lift.h"
#include "encoder.h"

Encoder encFront(enc_lift_frontA, enc_lift_frontB, 1.0);
Encoder encBack(enc_lift_backA, enc_lift_backB, 1.0);

void isrFrontA() { encFront.handleA(); }
void isrFrontB() { encFront.handleB(); }
void isrBackA()  { encBack.handleA(); }
void isrBackB()  { encBack.handleB(); }

unsigned long lastLogTime = 0;

void setup(){
    Serial.begin(115200);

    attachInterrupt(digitalPinToInterrupt(enc_lift_frontA), isrFrontA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_frontB), isrFrontB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_backA), isrBackA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_lift_backB), isrBackB, CHANGE);
}

void loop() {
    if (millis() - lastLogTime >= 100){
        Serial.print("Front: ");
        Serial.print(encFront.getCount());
        Serial.print(" | Back: ");
        Serial.println(encBack.getCount());
        lastLogTime = millis();
    }
}
