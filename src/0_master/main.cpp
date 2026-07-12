#include <Arduino.h>
#include "protocol.h"

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 50;

unsigned long lastServoSendTime = 0;
const unsigned long SERVO_SEND_INTERVAL_MS = 50;

unsigned long lastRelaySendTime = 0;
const unsigned long RELAY_SEND_INTERVAL_MS = 50;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial5.begin(115200);
}
void loop() {
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL_MS){
    sendWheelCommand(Serial1, 0.000f, 0.000f, 0.000f);
    lastSendTime = now;
  }

  if (now - lastServoSendTime >= SERVO_SEND_INTERVAL_MS){
    sendServoCommand(Serial5, 0, 0);
    lastServoSendTime = now;
  }

  if (now - lastRelaySendTime >= RELAY_SEND_INTERVAL_MS){
    sendRelayCommand(Serial5, 1, HIGH);
    sendRelayCommand(Serial5, 2, HIGH);
    sendRelayCommand(Serial5, 3, HIGH);
    sendRelayCommand(Serial5, 4, HIGH);

    lastRelaySendTime = now;
}
}