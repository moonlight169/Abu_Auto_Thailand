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
  Serial8.begin(115200);
}
void loop() {
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL_MS){
    sendWheelCommand(Serial1, 0.000f, 0.000f, 0.000f);
    lastSendTime = now;
  }

  if (now - lastServoSendTime >= SERVO_SEND_INTERVAL_MS){
    sendServoCommand(Serial8, 0, 0);
    lastServoSendTime = now;
  }

  if (now - lastRelaySendTime >= RELAY_SEND_INTERVAL_MS){
    sendRelayCommand(Serial8, 1, LOW);
    sendRelayCommand(Serial8, 2, LOW);
    sendRelayCommand(Serial8, 3, HIGH);
    sendRelayCommand(Serial8, 4, HIGH);

    lastRelaySendTime = now;
}
}