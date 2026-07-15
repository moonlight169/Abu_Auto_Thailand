#include <Arduino.h>
#include "protocol.h"
#include "TFminiS.h"
#include "laser_sensors.h"

TFminiS tofFront(Serial6);

#define WHEEL_SEND_HZ 20
#define TOF_READ_HZ 50 
#define SERVO_SEND_HZ 20  
#define RELAY_SEND_HZ 20    
#define ARM_SEND_HZ 20    
#define LASER_READ_HZ 2   
#define LIFT_SEND_HZ 2  

unsigned long lastSendTime = 0;
unsigned long lastTofReadTime = 0;
unsigned long lastServoSendTime = 0;
unsigned long lastRelaySendTime = 0;
unsigned long lastARMSendTime = 0;
unsigned long lastLaserReadTime = 0;
unsigned long lastLiftSendTime = 0;

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
  Serial8.begin(115200);

  //laser
  Serial4.begin(115200);
}
void loop() {
  unsigned long now = millis();

  if (now - lastSendTime >= (1000 / WHEEL_SEND_HZ)){
    sendWheelCommand(Serial1, 0.000f, 0.000f, 0.000f);
    lastSendTime = now;
  }

  //0, 1, 2, 3, 4
  if (now - lastARMSendTime >= (1000 / ARM_SEND_HZ)){
    sendArmCommand(Serial2, 0);
    lastARMSendTime = now;
  }

  if (now - lastServoSendTime >= (1000 / SERVO_SEND_HZ)){
    sendServoCommand(Serial8, 65, 0);
    lastServoSendTime = now;
  }

  if (now - lastRelaySendTime >= (1000 / RELAY_SEND_HZ)){
    sendRelayCommand(Serial8, 1, HIGH);
    sendRelayCommand(Serial8, 2, HIGH);
    sendRelayCommand(Serial8, 3, HIGH);
    sendRelayCommand(Serial8, 4, HIGH);

    lastRelaySendTime = now;
  }

  if (now - lastLiftSendTime >= (1000 / LIFT_SEND_HZ)){
    sendLiftCommandMM(Serial7, 0, 0);
    // sendLiftCommandMM(Serial7, 270, 270);
    lastLiftSendTime = now;
  }

  // 5_slave_laser เป็นบอร์ด Reader: ส่งข้อมูลเข้ามาเองทุก 50ms ทาง Serial8
  // เรียก readLaserCommand()/readLswCommand()/readLightCommand() ตอนไหนก็ได้เพื่อดึงค่าล่าสุดที่รับมา
  if (now - lastLaserReadTime >= (1000 / LASER_READ_HZ)){
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

  // if (now - lastTofReadTime >= (1000 / TOF_READ_HZ)){
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