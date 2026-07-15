#include <Arduino.h>
#include "protocol.h"
#include "TFminiS.h"

TFminiS tofFront(Serial6);

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 50;

unsigned long lastTofReadTime = 0;
const unsigned long TOF_READ_INTERVAL_MS = 20;

unsigned long lastServoSendTime = 0;
const unsigned long SERVO_SEND_INTERVAL_MS = 50;

unsigned long lastRelaySendTime = 0;
const unsigned long RELAY_SEND_INTERVAL_MS = 50;

unsigned long lastARMSendTime = 0;
const unsigned long ARM_SEND_INTERVAL_MS = 50;

void setup() {
  //monitor
  Serial.begin(115200);

  //wheels
  Serial1.begin(115200);

  //arm
  Serial2.begin(115200);

  // //lift
  // Serial7.begin(115200);

  // //laser
  // Serial4.begin(115200);

  // //sensor
  // Serial8.begin(115200);

  // //tof front
  // Serial6.begin(115200);
}
void loop() {
  unsigned long now = millis();


  if (now - lastSendTime >= SEND_INTERVAL_MS){
    sendWheelCommand(Serial1, 0.000f, 0.000f, 0.000f);
    lastSendTime = now;
  }

  //0, 1, 2, 3, 4
  if (now - lastARMSendTime >= ARM_SEND_INTERVAL_MS){
    sendArmCommand(Serial2, 0);
    lastARMSendTime = now;
  }

  // if (now - lastServoSendTime >= SERVO_SEND_INTERVAL_MS){
  //   sendServoCommand(Serial8, 0, 0);
  //   lastServoSendTime = now;
  // }

  // if (now - lastRelaySendTime >= RELAY_SEND_INTERVAL_MS){
  //   sendRelayCommand(Serial8, 1, LOW);
  //   sendRelayCommand(Serial8, 2, LOW);
  //   sendRelayCommand(Serial8, 3, HIGH);
  //   sendRelayCommand(Serial8, 4, HIGH);

  //   lastRelaySendTime = now;
  // }

  // if (now - lastTofReadTime >= TOF_READ_INTERVAL_MS){
  //   tofFront.readSensor();

  //   int distanceFront = tofFront.getDistance();
  //   if (distanceFront < 0){
  //     Serial.println(TFminiS::getErrorString(distanceFront));
  //   } else {
  //     Serial.print("TOF_Front: ");
  //     Serial.println(distanceFront);
  //   }

  //   lastTofReadTime = now;
  // }
}