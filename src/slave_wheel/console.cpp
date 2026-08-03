#include "console.h"

#include "encoder.h"
#include "master_link.h"
#include "speed_pid.h"

static char numberBuffer[16];
static uint8_t numberIndex = 0;

void printStatus()
{
  for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
  {
    Serial.print(WHEEL_NAME[wheel]);
    Serial.print(":SP=");
    Serial.print(targetRPM[wheel], 1);
    Serial.print(",RPM=");
    Serial.print(actualRPM[wheel], 1);
    Serial.print(",PWM=");
    Serial.print(pwmOutput[wheel]);
    Serial.print(",C=");
    Serial.print(encoderCount[wheel]);
    if (wheel < WHEEL_COUNT - 1)
      Serial.print(" | ");
  }
  Serial.println();
}

void printHelp()
{
  Serial.println("=== STM32 4-WHEEL MECANUM PID ===");
  Serial.println("100a : forward 100 RPM");
  Serial.println("100b : backward 100 RPM");
  Serial.println("100c : move left 100 RPM");
  Serial.println("100d : move right 100 RPM");
  Serial.println("s    : stop");
  Serial.println("v    : print wheel status");
  Serial.println("z    : zero all encoders (breaks Master odometry)");
  Serial.println("h    : help");
  Serial.println("No Enter needed: digits buffer, a letter runs at once.");
  Serial.println("Any letter drops Master control until its next T packet.");
  Serial.println("Number is wheel RPM, maximum 420 RPM.");
  Serial.println("From Master on Serial1: T,FL,FR,RL,RR | S | Q");
  Serial.println("No T packet for 500 ms -> stop and E,TIMEOUT_STOP");
}

static void executeSimpleCommand(char command)
{
  command = tolower(command);
  numberBuffer[numberIndex] = '\0';
  const float rpm =
      constrain((float)((numberIndex > 0) ? atol(numberBuffer) : 0),
                0.0f, MAX_RPM);
  numberIndex = 0;

  if (command == 'a')
  {
    setWheelTargets(rpm, rpm, rpm, rpm);
    Serial.print("FORWARD RPM=");
    Serial.println(rpm, 1);
  }
  else if (command == 'b')
  {
    setWheelTargets(-rpm, -rpm, -rpm, -rpm);
    Serial.print("BACKWARD RPM=");
    Serial.println(rpm, 1);
  }
  else if (command == 'c')
  {
    setWheelTargets(-rpm, rpm, rpm, -rpm);
    Serial.print("LEFT RPM=");
    Serial.println(rpm, 1);
  }
  else if (command == 'd')
  {
    setWheelTargets(rpm, -rpm, -rpm, rpm);
    Serial.print("RIGHT RPM=");
    Serial.println(rpm, 1);
  }
  else if (command == 's')
  {
    stopDrive();
    Serial.println("STOP");
  }
  else if (command == 'v')
  {
    printStatus();
  }
  else if (command == 'z')
  {
    noInterrupts();
    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
      encoderCount[wheel] = 0;
    interrupts();

    for (uint8_t wheel = 0; wheel < WHEEL_COUNT; wheel++)
      resetRPMFilter(wheel);

    Serial.println("ENCODERS ZERO");
  }
  else if (command == 'h')
  {
    printHelp();
  }
  else
  {
    Serial.println("ERR unknown command; use H");
  }
}

void readSimpleSerial()
{
  while (Serial.available())
  {
    const char received = Serial.read();

    if (received == '\r' || received == '\n' ||
        received == ' ' || received == '\t')
      continue;

    if (received >= '0' && received <= '9')
    {
      if (numberIndex < sizeof(numberBuffer) - 1)
        numberBuffer[numberIndex++] = received;
      continue;
    }

    if (isAlpha(received))
    {
      serial1ControlActive = false;
      executeSimpleCommand(received);
    }
  }
}
