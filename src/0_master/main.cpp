#include <Arduino.h>
#include "protocol.h"
#include "TFminiS.h"
#include "laser_sensors.h"

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

unsigned long lastLaserReadTime = 0;
const unsigned long LASER_READ_INTERVAL_MS = 500;

unsigned long lastLiftSendTime = 0;
const unsigned long LIFT_SEND_INTERVAL_MS = 500;

void setup() {
  //monitor
  Serial.begin(115200);

  //wheels
  Serial1.begin(115200);

  //arm
  Serial2.begin(115200);

  //lift
  Serial7.begin(115200);

  //sensor
  Serial4.begin(115200);

  //laser
  Serial8.begin(115200);
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

  if (now - lastServoSendTime >= SERVO_SEND_INTERVAL_MS){
    sendServoCommand(Serial4, 0, 0);
    lastServoSendTime = now;
  }

  if (now - lastRelaySendTime >= RELAY_SEND_INTERVAL_MS){
    sendRelayCommand(Serial4, 1, HIGH);
    sendRelayCommand(Serial4, 2, HIGH);
    sendRelayCommand(Serial4, 3, HIGH);
    sendRelayCommand(Serial4, 4, HIGH);

    lastRelaySendTime = now;
  }

  if (now - lastLiftSendTime >= LIFT_SEND_INTERVAL_MS){
    sendLiftCommandMM(Serial7, 0, 0);
    lastLiftSendTime = now;
  }

  // 5_slave_laser เป็นบอร์ด Reader: ส่งข้อมูลเข้ามาเองทุก 50ms ทาง Serial8
  // เรียก readLaserCommand()/readLswCommand()/readLightCommand() ตอนไหนก็ได้เพื่อดึงค่าล่าสุดที่รับมา
  if (now - lastLaserReadTime >= LASER_READ_INTERVAL_MS){
    lastLaserReadTime = now;

    LaserData laser = readLaserCommand();
    LswData lsw = readLswCommand();
    LightData light = readLightCommand();

    Serial.print("Laser: ");
    for (uint8_t i = 0; i < sizeof(laser.data); i++){
      Serial.print(laser.data[i]);
      Serial.print(' ');
    }

    Serial.print(" Lsw: ");
    Serial.print(lsw.data[0]);

    Serial.print(" Light: ");
    for (uint8_t i = 0; i < sizeof(light.data); i++){
      Serial.print(light.data[i]);
      Serial.print(' ');
    }
    Serial.println();
  }

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