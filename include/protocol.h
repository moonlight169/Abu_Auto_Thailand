#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <Arduino.h>

#define PROTOCOL_START_BYTE 0xAA
#define WHEEL_CMD_LEN sizeof(WheelCommand)
#define SERVO_CMD_LEN sizeof(ServoCommand)
#define SENSOR_LINK_BUFFER_SIZE (sizeof(ServoCommand) > sizeof(RelayCommand) ? sizeof(ServoCommand) : sizeof(RelayCommand))

#define CMD_SERVO 0x01
#define CMD_RELAY 0x02

struct WheelCommand{
    float vx;
    float vy;
    float omega;
};

struct ServoCommand{
    uint8_t armAngle;
    uint8_t spinAngle;
};

struct RelayCommand{
    uint8_t relayState;
};


struct WheelFrame{
    uint8_t start;
    uint8_t len;
    WheelCommand payload;
    uint8_t checksum;
};

enum ParserState{
    WAIT_START,
    READ_CMD_ID,
    READ_LEN,
    READ_PAYLOAD,
    READ_CHECKSUM
};

struct WheelReceiver{
    ParserState state = WAIT_START;
    uint8_t buffer[sizeof(WheelCommand)];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;
    WheelCommand lastCommand;
    bool hasNewCommand = false;
    unsigned long lastReceivedTime = 0;
};

struct SensorLinkReceiver{
    ParserState state = WAIT_START;
    uint8_t cmdId = 0;
    uint8_t buffer[SENSOR_LINK_BUFFER_SIZE];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;

    ServoCommand lastServoCommand;
    bool hasNewServoCommand = false;

    RelayCommand lastRelayCommand;
    bool hasNewRelayCommand = false;

    unsigned long lastReceivedTime = 0;
};

uint8_t calculateChecksum(const uint8_t* data, uint8_t len);
void wheelReceiverFeed(WheelReceiver &receiver, uint8_t incomingByte);
void sendWheelCommand(HardwareSerial &port, float vx, float vy, float omega);

void sensorLinkReceiverFeed(SensorLinkReceiver &receiver, uint8_t incomingByte);
void sendServoCommand(HardwareSerial &port, uint8_t armAngle, uint8_t spinAngle);
void sendRelayCommand(HardwareSerial &port, uint8_t relayNumber, uint8_t status);

#endif