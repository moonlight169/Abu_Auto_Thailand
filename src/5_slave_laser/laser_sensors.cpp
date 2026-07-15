#include "laser_sensors.h"
#include "config/config_laser.h"

static const int laserPins[5] = {
    Laser_1, Laser_2, Laser_3, Laser_4, Laser_5
};

static const int lightPins[2] = {
    Light_sensor_1, Light_sensor_2
};

static const uint8_t laserPinCount = sizeof(laserPins) / sizeof(laserPins[0]);
static const uint8_t lightPinCount = sizeof(lightPins) / sizeof(lightPins[0]);

void laserSensorsInit(){
    for (uint8_t i = 0; i < laserPinCount; i++){
        pinMode(laserPins[i], INPUT_PULLUP);
    }

    pinMode(L_SW_frontrobot_6, INPUT_PULLUP);
}

LaserData readLaserCommand(){
    LaserData result;
    for (uint8_t i = 0; i < laserPinCount; i++){
        result.header[i] = i;
        result.data[i] = digitalRead(laserPins[i]);
    }
    return result;
}

LswData readLswCommand(){
    LswData result;
    result.header[0] = 0;
    result.data[0] = digitalRead(L_SW_frontrobot_6);
    return result;
}

LightData readLightCommand(){
    LightData result;
    for (uint8_t i = 0; i < lightPinCount; i++){
        result.header[i] = i;
        result.data[i] = (uint8_t)analogRead(lightPins[i]);
    }
    return result;
}
