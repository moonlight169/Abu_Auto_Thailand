#ifndef LASER_SENSORS_H
#define LASER_SENSORS_H

#include <Arduino.h>
#include "protocol.h"

void laserSensorsInit();

LaserData readLaserCommand();
LswData readLswCommand();
LightData readLightCommand();

#endif
