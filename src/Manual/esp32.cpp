#include <Arduino.h>
#include <PS4Controller.h>
#include <string.h>
#include "protocol.h"

#define MANUAL_SEND_HZ 50
#define STICK_DEADZONE 10

#define MAX_LINEAR_SPEED 1.0f   // m/s ความเร็วเชิงเส้นสูงสุดตอน manual
#define MAX_ANGULAR_SPEED 2.0f  // rad/s ความเร็วหมุนสูงสุดตอน manual

unsigned long lastSendTime = 0;

static uint8_t calcChecksum(const uint8_t* data, uint8_t len){
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++){
    checksum ^= data[i];
  }
  return checksum;
}

static void sendManualWheelCommand(float vx, float vy, float omega){
  WheelCommand cmd;
  cmd.vx = vx;
  cmd.vy = vy;
  cmd.omega = omega;

  uint8_t payloadBytes[sizeof(WheelCommand)];
  memcpy(payloadBytes, &cmd, sizeof(WheelCommand));

  uint8_t checksum = calcChecksum(payloadBytes, sizeof(WheelCommand));

  Serial2.write(PROTOCOL_START_BYTE);
  Serial2.write((uint8_t)sizeof(WheelCommand));
  Serial2.write(payloadBytes, sizeof(WheelCommand));
  Serial2.write(checksum);
}

static float applyDeadzone(int8_t rawStick){
  if (rawStick > -STICK_DEADZONE && rawStick < STICK_DEADZONE){
    return 0.0f;
  }
  return (float)rawStick / 127.0f;
}

void setup(){
  Serial.begin(115200);

  Serial2.begin(115200);

  PS4.begin("c0:cd:d6:8d:0a:64");
  Serial.println("Waiting for PS4 controller...");
}

void loop(){
  unsigned long now = millis();

  if (now - lastSendTime >= (1000 / MANUAL_SEND_HZ)){
    lastSendTime = now;

    if (PS4.isConnected()){
      float vx = applyDeadzone(PS4.LStickY()) * MAX_LINEAR_SPEED;
      float vy = applyDeadzone(PS4.RStickX()) * MAX_LINEAR_SPEED;
      float omega = applyDeadzone(PS4.LStickX()) * MAX_ANGULAR_SPEED;

      sendManualWheelCommand(vx, vy, omega);

      Serial.print("vx: ");
      Serial.print(vx);
      Serial.print(" vy: ");
      Serial.print(vy);
      Serial.print(" omega: ");
      Serial.println(omega);
    } else {
      sendManualWheelCommand(0.0f, 0.0f, 0.0f);
    }
  }
}
