#include <string.h>
#include "protocol.h"

uint8_t calculateChecksum(const uint8_t* data, uint8_t len){
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++){
        checksum ^= data[i];
    }
    return checksum;
}

void sendLaserData(HardwareSerial &port, const LaserData &laser){
    uint8_t payloadBytes[sizeof(LaserData)];
    memcpy(payloadBytes, &laser, sizeof(LaserData));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(LaserData));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_LASER);
    port.write((uint8_t)sizeof(LaserData));
    port.write(payloadBytes, sizeof(LaserData));
    port.write(checksum);
}

void sendLswData(HardwareSerial &port, const LswData &lsw){
    uint8_t payloadBytes[sizeof(LswData)];
    memcpy(payloadBytes, &lsw, sizeof(LswData));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(LswData));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_LSW);
    port.write((uint8_t)sizeof(LswData));
    port.write(payloadBytes, sizeof(LswData));
    port.write(checksum);
}

void sendLightData(HardwareSerial &port, const LightData &light){
    uint8_t payloadBytes[sizeof(LightData)];
    memcpy(payloadBytes, &light, sizeof(LightData));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(LightData));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_LIGHT);
    port.write((uint8_t)sizeof(LightData));
    port.write(payloadBytes, sizeof(LightData));
    port.write(checksum);
}

void sendButtonData(HardwareSerial &port, const ButtonData &button){
    uint8_t payloadBytes[sizeof(ButtonData)];
    memcpy(payloadBytes, &button, sizeof(ButtonData));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(ButtonData));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_BUTTON);
    port.write((uint8_t)sizeof(ButtonData));
    port.write(payloadBytes, sizeof(ButtonData));
    port.write(checksum);
}
