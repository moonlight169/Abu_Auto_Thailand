#include <string.h>
#include "protocol.h"

uint8_t calculateChecksum(const uint8_t* data, uint8_t len){
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++){
        checksum ^= data[i];
    }
    return checksum;
}

void sendArmFeedback(HardwareSerial &port, uint8_t bottomAngle, uint8_t topAngle, uint8_t status){
    ArmFeedback fb;
    fb.bottomAngle = bottomAngle;
    fb.topAngle = topAngle;
    fb.status = status;

    uint8_t payloadBytes[sizeof(ArmFeedback)];
    memcpy(payloadBytes, &fb, sizeof(ArmFeedback));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(ArmFeedback));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_ARM_FB);
    port.write((uint8_t)sizeof(ArmFeedback));
    port.write(payloadBytes, sizeof(ArmFeedback));
    port.write(checksum);
}

void armReceiverFeed(ArmReceiver &receiver, uint8_t incomingByte){
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
            if (receiver.expectedLen > 0 && receiver.expectedLen <= ARM_CMD_LEN){
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
                if (receiver.cmdId == CMD_ARM && receiver.expectedLen == sizeof(ArmCommand)){
                    memcpy(&receiver.lastArmCommand, receiver.buffer, sizeof(ArmCommand));
                    receiver.hasNewArmCommand = true;
                    receiver.lastReceivedTime = millis();
                }
                else if (receiver.cmdId == CMD_ARM_CODE && receiver.expectedLen == sizeof(ArmCodeCommand)){
                    receiver.lastArmCode = receiver.buffer[0];
                    receiver.hasNewArmCode = true;
                    receiver.lastReceivedTime = millis();
                }
            }
            receiver.state = WAIT_START;
            break;
        }
    }
}