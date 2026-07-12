#include <Arduino.h>
#include "protocol.h"

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 50;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}
void loop() {
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL_MS){
    sendWheelCommand(Serial1, 0.000f, 0.600f, 0.000f);
    lastSendTime = now;
  }
}