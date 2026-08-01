#include "tfmini.h"

TfminiData tfminiData[TFMINI_COUNT] = {};

void beginTfmini() {
  tfmini1.begin(TFMINI_BAUD);
  tfmini2.begin(TFMINI_BAUD);
  tfmini3.begin(TFMINI_BAUD);
  tfmini4.begin(TFMINI_BAUD);
}

bool tfminiFresh(uint8_t index) {
  return index < TFMINI_COUNT &&
         tfminiData[index].valid &&
         millis() - tfminiData[index].lastFrameMs <= TFMINI_TIMEOUT_MS;
}

uint8_t getTfminiValidMask() {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < TFMINI_COUNT; i++) {
    if (tfminiFresh(i)) {
      mask |= (1U << i);
    }
  }
  return mask;
}

static void readTfminiPort(Stream &port, TfminiData &sensor) {
  while (port.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(port.read());

    // Every valid TFMini-S frame starts with 0x59, 0x59.
    if (sensor.index == 0) {
      if (value == 0x59) {
        sensor.frame[sensor.index++] = value;
      }
      continue;
    }

    if (sensor.index == 1) {
      if (value == 0x59) {
        sensor.frame[sensor.index++] = value;
      } else {
        sensor.index = 0;
      }
      continue;
    }

    sensor.frame[sensor.index++] = value;
    if (sensor.index < sizeof(sensor.frame)) {
      continue;
    }

    sensor.index = 0;
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 8; i++) {
      checksum += sensor.frame[i];
    }

    if (checksum != sensor.frame[8]) {
      continue;
    }

    const uint16_t distance =
        uint16_t(sensor.frame[2]) | (uint16_t(sensor.frame[3]) << 8);
    const uint16_t strength =
        uint16_t(sensor.frame[4]) | (uint16_t(sensor.frame[5]) << 8);

    // TFMini-S distance output is in centimetres.
    if (distance == 0 || distance > 1200) {
      continue;
    }

    sensor.distanceCm = distance;
    sensor.strength = strength;
    sensor.lastFrameMs = millis();
    sensor.valid = true;
  }
}

void updateTfmini() {
  readTfminiPort(tfmini1, tfminiData[0]);
  readTfminiPort(tfmini2, tfminiData[1]);
  readTfminiPort(tfmini3, tfminiData[2]);
  readTfminiPort(tfmini4, tfminiData[3]);

  for (uint8_t i = 0; i < TFMINI_COUNT; i++) {
    if (tfminiData[i].valid &&
        millis() - tfminiData[i].lastFrameMs > TFMINI_TIMEOUT_MS) {
      tfminiData[i].valid = false;
    }
  }
}

void printTfminiMonitor()
{
  static uint32_t lastPrintMs = 0;

  if (millis() - lastPrintMs < 200)
    return;

  lastPrintMs = millis();

  Serial.print("TF1=");
  if (tfminiFresh(0))
    Serial.print(tfminiData[0].distanceCm);
  else
    Serial.print("NO DATA");

  Serial.print(" cm | TF2=");
  if (tfminiFresh(1))
    Serial.print(tfminiData[1].distanceCm);
  else
    Serial.print("NO DATA");

  Serial.print(" cm | TF3=");
  if (tfminiFresh(2))
    Serial.print(tfminiData[2].distanceCm);
  else
    Serial.print("NO DATA");

  Serial.print(" cm | TF4=");
  if (tfminiFresh(3))
    Serial.print(tfminiData[3].distanceCm);
  else
    Serial.print("NO DATA");

  Serial.print(" cm | validMask=");
  Serial.println(getTfminiValidMask());
}
