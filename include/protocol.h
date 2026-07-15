#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <Arduino.h>

#define PROTOCOL_START_BYTE 0xAA
#define WHEEL_CMD_LEN sizeof(WheelCommand)
#define SERVO_CMD_LEN sizeof(ServoCommand)
#define SENSOR_LINK_BUFFER_SIZE (sizeof(ServoCommand) > sizeof(RelayCommand) ? sizeof(ServoCommand) : sizeof(RelayCommand))
#define ARM_CMD_LEN sizeof(ArmCommand)
#define ARM_FB_LEN sizeof(ArmFeedback)
#define LASER_LINK_BUFFER_SIZE sizeof(LaserData)  // LaserData(10B) เป็น payload ที่ใหญ่ที่สุดในกลุ่ม Laser/Lsw/Light

#define CMD_SERVO 0x01
#define CMD_RELAY 0x02
#define CMD_ARM   0x03
#define CMD_ARM_FB 0x04
#define CMD_ARM_CODE 0x05
#define CMD_LASER 0x06
#define CMD_LSW   0x07
#define CMD_LIGHT 0x08

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

struct ArmCommand{
    uint8_t bottomAngle;
    uint8_t topAngle; 
    uint8_t flags;      
};

struct ArmCodeCommand{
    uint8_t code;   // รหัสท่าของ 2_slave_arm (ตัวเลข):
                    // 0=bottom 0°, 1=bottom 180°, 2=top center,
                    // 3=top +90°, 4=top -90°, 9=calibrate bottom
};

struct ArmFeedback{
    uint8_t bottomAngle;
    uint8_t topAngle;
    uint8_t status;
};

struct LaserData{
    uint8_t header[5];  // ลำดับเซนเซอร์ 0-4
    uint8_t data[5];    // ผลอ่าน digitalRead ต่อเซนเซอร์ (0/1)
};

struct LswData{
    uint8_t header[1];
    uint8_t data[1];    // ผลอ่าน digitalRead ของ L_SW_frontrobot_6
};

struct LightData{
    uint8_t header[2];
    uint8_t data[2];    // ผลอ่าน analogRead (ตั้ง analogReadResolution(8) ไว้ที่ 0-255)
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

struct ArmReceiver{
    ParserState state = WAIT_START;
    uint8_t cmdId = 0;
    uint8_t buffer[ARM_CMD_LEN];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;

    ArmCommand lastArmCommand;
    bool hasNewArmCommand = false;

    uint8_t lastArmCode = 0;
    bool hasNewArmCode = false;

    unsigned long lastReceivedTime = 0;
};

struct ArmFeedbackReceiver{
    ParserState state = WAIT_START;
    uint8_t cmdId = 0;
    uint8_t buffer[ARM_FB_LEN];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;

    ArmFeedback lastFeedback;
    bool hasNewFeedback = false;

    unsigned long lastReceivedTime = 0;
};

struct LaserLinkReceiver{
    ParserState state = WAIT_START;
    uint8_t cmdId = 0;
    uint8_t buffer[LASER_LINK_BUFFER_SIZE];
    uint8_t bufferIndex = 0;
    uint8_t expectedLen = 0;

    LaserData lastLaser;
    bool hasNewLaser = false;

    LswData lastLsw;
    bool hasNewLsw = false;

    LightData lastLight;
    bool hasNewLight = false;

    unsigned long lastReceivedTime = 0;
};

uint8_t calculateChecksum(const uint8_t* data, uint8_t len);
void wheelReceiverFeed(WheelReceiver &receiver, uint8_t incomingByte);
void sendWheelCommand(HardwareSerial &port, float vx, float vy, float omega);

void sensorLinkReceiverFeed(SensorLinkReceiver &receiver, uint8_t incomingByte);
void sendServoCommand(HardwareSerial &port, uint8_t armAngle, uint8_t spinAngle);
void sendRelayCommand(HardwareSerial &port, uint8_t relayNumber, uint8_t status);

void armReceiverFeed(ArmReceiver &receiver, uint8_t incomingByte);
void sendArmCommand(HardwareSerial &port, uint8_t code);

void armFeedbackReceiverFeed(ArmFeedbackReceiver &receiver, uint8_t incomingByte);
void sendArmFeedback(HardwareSerial &port, uint8_t bottomAngle, uint8_t topAngle, uint8_t status);

void sendLaserData(HardwareSerial &port, const LaserData &laser);
void sendLswData(HardwareSerial &port, const LswData &lsw);
void sendLightData(HardwareSerial &port, const LightData &light);

void laserLinkReceiverFeed(LaserLinkReceiver &receiver, uint8_t incomingByte);

#endif