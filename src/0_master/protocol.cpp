#include <string.h>
#include "protocol.h"

static uint8_t currentRelayState = 0x00;

uint8_t calculateChecksum(const uint8_t* data, uint8_t len){
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++){
        checksum ^= data[i];
    }
    return checksum;
}

void sendWheelCommand(HardwareSerial &port, float vx, float vy, float omega){
    WheelCommand cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.omega = omega;

    uint8_t payloadBytes[sizeof(WheelCommand)];
    memcpy(payloadBytes, &cmd, sizeof(WheelCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(WheelCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)sizeof(WheelCommand));
    port.write(payloadBytes, sizeof(WheelCommand));
    port.write(checksum);
}

void sendServoCommand(HardwareSerial &port, uint8_t armAngle, uint8_t spinAngle){
    ServoCommand cmd;
    cmd.armAngle = armAngle;
    cmd.spinAngle = spinAngle;

    uint8_t payloadBytes[sizeof(ServoCommand)];
    memcpy(payloadBytes, &cmd, sizeof(ServoCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(ServoCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_SERVO);
    port.write((uint8_t)sizeof(ServoCommand));
    port.write(payloadBytes, sizeof(ServoCommand));
    port.write(checksum);
}

void sendRelayCommand(HardwareSerial &port, uint8_t relayNumber, uint8_t status){
    if (relayNumber < 1 || relayNumber > 4){
        return;
    }

    uint8_t bitMask = 1 << (relayNumber - 1);

    if (status == LOW){                        // Active LOW: LOW = relay ON
        currentRelayState |= bitMask;
    } else {
        currentRelayState &= ~bitMask;
    }

    RelayCommand cmd;
    cmd.relayState = currentRelayState;

    uint8_t payloadBytes[sizeof(RelayCommand)];
    memcpy(payloadBytes, &cmd, sizeof(RelayCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(RelayCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_RELAY);
    port.write((uint8_t)sizeof(RelayCommand));
    port.write(payloadBytes, sizeof(RelayCommand));
    port.write(checksum);
}