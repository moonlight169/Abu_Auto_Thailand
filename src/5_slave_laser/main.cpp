#include <Arduino.h>
#include "protocol.h"
#include "laser_sensors.h"

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 5;

unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL_MS = 500;

void setup(){
    Serial.begin(115200);
    Serial1.begin(115200);

    analogReadResolution(8);   // ให้ analogRead คืนค่า 0-255 ตรงกับ LightData.data (uint8_t)
    laserSensorsInit();

    Serial.println(F("=== Slave Laser Ready ==="));
}

void loop(){
    // ===== 1. อ่านเซนเซอร์ทั้งหมดทุกรอบลูป (non-blocking, ไม่มี delay) =====
    LaserData laser = readLaserCommand();
    LswData lsw = readLswCommand();
    LightData light = readLightCommand();
    ButtonData button = readButtonCommand();

    // ===== 2. ส่งค่าล่าสุดกลับ master เป็นช่วงๆ =====
    if (millis() - lastSendTime >= SEND_INTERVAL_MS){
        lastSendTime = millis();

        sendLaserData(Serial1, laser);
        sendLswData(Serial1, lsw);
        sendLightData(Serial1, light);
        sendButtonData(Serial1, button);
    }

    // ===== 3. Debug ผ่าน USB Serial =====
    if (millis() - lastLogTime >= LOG_INTERVAL_MS){
        lastLogTime = millis();

        Serial.print(F("Laser: "));
        for (uint8_t i = 0; i < sizeof(laser.data); i++){
            Serial.print(laser.data[i]);
            Serial.print(' ');
        }

        Serial.print(F(" Lsw: "));
        Serial.print(lsw.data[0]);

        Serial.print(F(" Light: "));
        for (uint8_t i = 0; i < sizeof(light.data); i++){
            Serial.print(light.data[i]);
            Serial.print(' ');
        }

        Serial.print(F(" Button: "));
        for (uint8_t i = 0; i < sizeof(button.data); i++){
            Serial.print(button.data[i]);
            Serial.print(' ');
        }
        Serial.println();
    }
}
