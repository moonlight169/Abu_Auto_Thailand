#include "laser_sensors.h"

static LaserLinkReceiver laserReceiver;

// ดึงไบต์ที่มาจาก 5_slave_laser (Serial8) เข้ามาป้อน parser ให้ทันก่อนคืนค่าล่าสุด
static void laserLinkPump(){
    while (Serial8.available() > 0){
        laserLinkReceiverFeed(laserReceiver, Serial8.read());
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
