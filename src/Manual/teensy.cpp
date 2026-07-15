#include <Arduino.h>
#include <string.h>
#include "protocol.h"

#define MANUAL_CMD_TIMEOUT_MS 300
#define PRINT_HZ 20
#define WHEEL_SEND_HZ 50

WheelReceiver wheelReceiver;

unsigned long lastSendTime = 0;
unsigned long g_prevCommandTime = 0;
unsigned long lastPrintTime = 0;

static uint8_t calcChecksum(const uint8_t* data, uint8_t len){
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++){
    checksum ^= data[i];
  }
  return checksum;
}

// state machine เดียวกับ wheelReceiverFeed ใน protocol.cpp ของ 1_slave_wheel
// เขียนแยกในนี้เพราะ Manual/protocol.cpp เป็นคนละเวอร์ชัน (ของฝั่ง laser board) ไม่มีฟังก์ชันนี้
static void feedWheelReceiver(WheelReceiver &receiver, uint8_t incomingByte){
  switch (receiver.state){
    case WAIT_START:
      if (incomingByte == PROTOCOL_START_BYTE){
        receiver.state = READ_LEN;
      }
      break;

    case READ_LEN:
      receiver.expectedLen = incomingByte;
      receiver.bufferIndex = 0;
      if (receiver.expectedLen == WHEEL_CMD_LEN){
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
      uint8_t expectedChecksum = calcChecksum(receiver.buffer, receiver.expectedLen);
      if (incomingByte == expectedChecksum){
        memcpy(&receiver.lastCommand, receiver.buffer, sizeof(WheelCommand));
        receiver.hasNewCommand = true;
        receiver.lastReceivedTime = millis();
      }
      receiver.state = WAIT_START;
      break;
    }
  }
}

void setup(){
  Serial.begin(115200);

  Serial2.begin(115200);

  //esp32
  Serial5.begin(115200);
}

void loop(){
  while (Serial5.available() > 0){
    uint8_t incomingByte = Serial5.read();
    feedWheelReceiver(wheelReceiver, incomingByte);
  }

  if (wheelReceiver.hasNewCommand){
    g_prevCommandTime = millis();
    wheelReceiver.hasNewCommand = false;
  }

  unsigned long now = millis();
  bool linkTimedOut = (now - g_prevCommandTime > MANUAL_CMD_TIMEOUT_MS);

  if (now - lastPrintTime >= (1000 / PRINT_HZ)){
    lastPrintTime = now;

    if (linkTimedOut){
      Serial.println("Manual link: no data (timeout)");
    } else {
      Serial.print("vx: ");
      Serial.print(wheelReceiver.lastCommand.vx);
      Serial.print(" vy: ");
      Serial.print(wheelReceiver.lastCommand.vy);
      Serial.print(" omega: ");
      Serial.println(wheelReceiver.lastCommand.omega);
    }
  }

  if (now - lastSendTime >= (1000 / WHEEL_SEND_HZ)){
    if (linkTimedOut){
      sendWheelCommand(Serial1, 0.000f, 0.000f, 0.000f);
    } else {
      sendWheelCommand(Serial1, wheelReceiver.lastCommand.vx, wheelReceiver.lastCommand.vy, wheelReceiver.lastCommand.omega);
    }
    lastSendTime = now;
  }
}
