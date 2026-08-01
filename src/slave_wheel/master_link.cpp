#include "master_link.h"

#include "encoder.h"
#include "speed_pid.h"

bool serial1ControlActive = false;

static char serial1RxBuffer[SERIAL1_RX_BUFFER_SIZE];
static uint8_t serial1RxIndex = 0;

void sendSerial1Feedback()
{
  int32_t countSnapshot[WHEEL_COUNT];

  noInterrupts();
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
    countSnapshot[wheel] = encoderCount[wheel];
  interrupts();

  Serial1.print("F,");
  Serial1.print(millis());

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    Serial1.print(',');
    Serial1.print(countSnapshot[wheel]);
  }

  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    Serial1.print(',');
    Serial1.print(actualRPM[wheel], 1);
  }

  Serial1.println();
}

static void processSerial1Command(char *line)
{
  char *originalLine = line;

  while (*line == ' ' || *line == '\t')
    line++;

  if (*line == 'T' || *line == 't')
  {
    float values[WHEEL_COUNT];
    char *cursor = line + 1;
    bool valid = true;

    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
    {
      // Accept either T,100,100,100,100 or T 100 100 100 100.
      while (*cursor == ',' || *cursor == ' ' || *cursor == '\t')
        cursor++;

      char *endPointer;
      long rpm = strtol(cursor, &endPointer, 10);
      if (endPointer == cursor)
      {
        valid = false;
        break;
      }
      values[wheel] = constrain((float)rpm, -MAX_RPM, MAX_RPM);
      cursor = endPointer;
    }

    while (*cursor == ',' || *cursor == ' ' || *cursor == '\t')
      cursor++;

    if (valid && *cursor == '\0')
    {
      setWheelTargets(values[FL], values[FR],
                      values[RL], values[RR]);
      serial1ControlActive = true;
      lastCommandTime = millis();
      return;
    }

    Serial1.print("E,BAD_TARGET,RX=[");
    Serial1.print(originalLine);
    Serial1.println("]");
    return;
  }

  if ((*line == 'S' || *line == 's') && line[1] == '\0')
  {
    stopDrive();
    serial1ControlActive = false;
    Serial1.println("A,STOP");
    return;
  }

  if ((*line == 'Q' || *line == 'q') && line[1] == '\0')
  {
    sendSerial1Feedback();
    return;
  }

  Serial1.println("E,UNKNOWN");
}

void readSerial1Commands()
{
  while (Serial1.available() > 0)
  {
    const char received = (char)Serial1.read();

    if (received == '\n' || received == '\r')
    {
      if (serial1RxIndex > 0)
      {
        serial1RxBuffer[serial1RxIndex] = '\0';
        processSerial1Command(serial1RxBuffer);
        serial1RxIndex = 0;
      }
      continue;
    }

    if (serial1RxIndex < SERIAL1_RX_BUFFER_SIZE - 1)
    {
      serial1RxBuffer[serial1RxIndex++] = received;
    }
    else
    {
      serial1RxIndex = 0;
      Serial1.println("E,OVERFLOW");
    }
  }
}
