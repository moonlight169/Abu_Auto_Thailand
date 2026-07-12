#include <string.h>
#include "protocol.h"

uint8_t calculateChecksum(const uint8_t* data, uint8_t len){
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++){
        checksum ^= data[i];
    }
    return checksum;
}

void sensorLinkReceiverFeed(SensorLinkReceiver &receiver, uint8_t incomingByte){
    switch (receiver.state){
        case WAIT_START:
            if (incomingByte == PROTOCOL_START_BYTE){
                receiver.state = READ_CMD_ID;
            }
            break;

        case READ_CMD_ID:
            receiver.cmdId = incomingByte;
            receiver.state = READ_LEN;
            break;

        case READ_LEN:
            receiver.expectedLen = incomingByte;
            receiver.bufferIndex = 0;
            if (receiver.expectedLen > 0 && receiver.expectedLen <= SENSOR_LINK_BUFFER_SIZE){
                receiver.state = READ_PAYLOAD;
            } else {
                receiver.state = WAIT_START;
            }
            break;

        case READ_PAYLOAD:
            receiver.buffer[receiver.bufferIndex] = incomingByte;
            receiver.bufferIndex++;
            if (receiver.bufferIndex >= receiver.expectedLen){
                receiver.state = READ_CHECKSUM;
            }
            break;

        case READ_CHECKSUM: {
            uint8_t expectedChecksum = calculateChecksum(receiver.buffer, receiver.expectedLen);
            if (incomingByte == expectedChecksum){
                if (receiver.cmdId == CMD_SERVO && receiver.expectedLen == sizeof(ServoCommand)){
                    memcpy(&receiver.lastServoCommand, receiver.buffer, sizeof(ServoCommand));
                    receiver.hasNewServoCommand = true;
                    receiver.lastReceivedTime = millis();
                } else if (receiver.cmdId == CMD_RELAY && receiver.expectedLen == sizeof(RelayCommand)){
                    memcpy(&receiver.lastRelayCommand, receiver.buffer, sizeof(RelayCommand));
                    receiver.hasNewRelayCommand = true;
                    receiver.lastReceivedTime = millis();
                }
            }
            receiver.state = WAIT_START;
            break;
        }
    }
}