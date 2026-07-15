#include "laser_sensors.h"

static LaserLinkReceiver laserReceiver;

static void laserLinkPump(){
    while (Serial4.available() > 0){
        laserLinkReceiverFeed(laserReceiver, Serial4.read());
    }
}

void laserSensorsInit(){
    // master ไม่มีเซนเซอร์ต่อตรง เก็บฟังก์ชันนี้ไว้ให้ตรงกับ interface เดียวกับฝั่ง 5_slave_laser
}

LaserData readLaserCommand(){
    laserLinkPump();
    return laserReceiver.lastLaser;
}

LswData readLswCommand(){
    laserLinkPump();
    return laserReceiver.lastLsw;
}

LightData readLightCommand(){
    laserLinkPump();
    return laserReceiver.lastLight;
}
