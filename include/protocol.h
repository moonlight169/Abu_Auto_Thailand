#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <Arduino.h>

#define PROTOCOL_START_BYTE 0xAA
#define WHEEL_CMD_LEN sizeof(WheelCommand)

struct WheelCommand{
    float vx;
    float vy;
    float omega;
};

struct WheelFrame{
    uint8_t start;
    uint8_t len;
    WheelCommand payload;
    uint8_t checksum;
};

enum ParserState{
    WAIT_START,
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

uint8_t calculateChecksum(const uint8_t* data, uint8_t len);
void wheelReceiverFeed(WheelReceiver &receiver, uint8_t incomingByte);
void sendWheelCommand(HardwareSerial &port, float vx, float vy, float omega);


#endif