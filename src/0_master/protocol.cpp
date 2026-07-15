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

void sendArmCommand(HardwareSerial &port, uint8_t code){
    ArmCodeCommand cmd;
    cmd.code = code;

    uint8_t payloadBytes[sizeof(ArmCodeCommand)];
    memcpy(payloadBytes, &cmd, sizeof(ArmCodeCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(ArmCodeCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_ARM_CODE);
    port.write((uint8_t)sizeof(ArmCodeCommand));
    port.write(payloadBytes, sizeof(ArmCodeCommand));
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

void laserLinkReceiverFeed(LaserLinkReceiver &receiver, uint8_t incomingByte){
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
            if (receiver.expectedLen > 0 && receiver.expectedLen <= LASER_LINK_BUFFER_SIZE){
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
                if (receiver.cmdId == CMD_LASER && receiver.expectedLen == sizeof(LaserData)){
                    memcpy(&receiver.lastLaser, receiver.buffer, sizeof(LaserData));
                    receiver.hasNewLaser = true;
                    receiver.lastReceivedTime = millis();
                } else if (receiver.cmdId == CMD_LSW && receiver.expectedLen == sizeof(LswData)){
                    memcpy(&receiver.lastLsw, receiver.buffer, sizeof(LswData));
                    receiver.hasNewLsw = true;
                    receiver.lastReceivedTime = millis();
                } else if (receiver.cmdId == CMD_LIGHT && receiver.expectedLen == sizeof(LightData)){
                    memcpy(&receiver.lastLight, receiver.buffer, sizeof(LightData));
                    receiver.hasNewLight = true;
                    receiver.lastReceivedTime = millis();
                }
            }
            receiver.state = WAIT_START;
            break;
        }
    }
}

void sendLiftCommandPulse(HardwareSerial &port, long pulseFront, long pulseBack){
    LiftPulseCommand cmd;
    cmd.pulseFront = pulseFront;
    cmd.pulseBack = pulseBack;

    uint8_t payloadBytes[sizeof(LiftPulseCommand)];
    memcpy(payloadBytes, &cmd, sizeof(LiftPulseCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(LiftPulseCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_LIFT_PULSE);
    port.write((uint8_t)sizeof(LiftPulseCommand));
    port.write(payloadBytes, sizeof(LiftPulseCommand));
    port.write(checksum);
}

void sendLiftCommandMM(HardwareSerial &port, float mmFront, float mmBack){
    LiftMMCommand cmd;
    cmd.mmFront = mmFront;
    cmd.mmBack = mmBack;

    uint8_t payloadBytes[sizeof(LiftMMCommand)];
    memcpy(payloadBytes, &cmd, sizeof(LiftMMCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(LiftMMCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_LIFT_MM);
    port.write((uint8_t)sizeof(LiftMMCommand));
    port.write(payloadBytes, sizeof(LiftMMCommand));
    port.write(checksum);
}

void sendLiftCommandZero(HardwareSerial &port){
    LiftZeroCommand cmd;
    cmd.trigger = 1;

    uint8_t payloadBytes[sizeof(LiftZeroCommand)];
    memcpy(payloadBytes, &cmd, sizeof(LiftZeroCommand));

    uint8_t checksum = calculateChecksum(payloadBytes, sizeof(LiftZeroCommand));

    port.write(PROTOCOL_START_BYTE);
    port.write((uint8_t)CMD_LIFT_ZERO);
    port.write((uint8_t)sizeof(LiftZeroCommand));
    port.write(payloadBytes, sizeof(LiftZeroCommand));
    port.write(checksum);
}