#include "console.h"

#include "arm_control.h"
#include "encoder.h"
#include "motor.h"

bool limitStreamEnabled = false;

static String serialLine;

void initializeConsole() {
  serialLine.reserve(40);
}

void printHelp() {
  Serial.println();
  Serial.println(F("=== ABU 2026 ARM SERIAL TEST ==="));
  Serial.println(F("HOME        : Bottom FRONT=0 -> Top FRONT=0 -> Top moves to 100 deg"));
  Serial.println(F("B<number>   : Bottom PID position 0..180, e.g. B90 or B45.5"));
  Serial.println(F("T<number>   : Top position 0..180 deg, e.g. T90 or T45.5"));
  Serial.println(F("BPWM n      : Test Bottom PWM -255..255 (-=FRONT, +=BACK)"));
  Serial.println(F("TPWM n      : Test Top PWM -255..255 (+=FRONT, -=BACK)"));
  Serial.println(F("LIMITS or L : Read all four limits once (PRESSED/OPEN)"));
  Serial.println(F("STREAM ON   : Print limits continuously every 200 ms"));
  Serial.println(F("STREAM OFF  : Back to STATUS every 500 ms"));
  Serial.println(F("STOP or X   : Stop both motors"));
  Serial.println(F("STATUS or ? : Print encoder, position, limits, state"));
  Serial.println(F("CLEAR       : Clear FAULT and return to IDLE"));
  Serial.println(F("HELP        : Show this menu"));
  Serial.println(F("No comma here: B90 means 90 deg, B,90 parses as 0 deg."));
  Serial.println(F("The Master uses its own CSV parser: CMD,<seq>,B,90"));
  Serial.println();
}

void printStatus() {
  Serial.print(F("[STATUS] state="));
  Serial.print(stateName());
  Serial.print(F(" bottomHomed="));
  Serial.print(bottomHomed ? F("YES") : F("NO"));
  Serial.print(F(" topHomed="));
  Serial.print(topHomed ? F("YES") : F("NO"));
  Serial.print(F(" bottomPulse="));
  Serial.print(readBottomEncoder());
  Serial.print(F(" bottomDeg="));
  Serial.print(getBottomPositionDeg(), 2);
  Serial.print(F(" bottomTarget="));
  Serial.print(bottomTargetDeg, 1);
  Serial.print(F(" topPulse="));
  Serial.print(readTopEncoder());
  Serial.print(F(" topDeg="));
  Serial.print(getTopPositionDeg(), 2);
  Serial.print(F(" target="));
  Serial.print(topTargetDeg, 1);
  Serial.print(F(" | limits TF="));
  Serial.print(isPressed(TOP_LIMIT_FRONT));
  Serial.print(F(" TB="));
  Serial.print(isPressed(TOP_LIMIT_BACK));
  Serial.print(F(" BF="));
  Serial.print(isPressed(BOTTOM_LIMIT_FRONT));
  Serial.print(F(" BB="));
  Serial.println(isPressed(BOTTOM_LIMIT_BACK));
}

void printLimits() {
  Serial.print(F("[LIMITS] TopFront="));
  Serial.print(isPressed(TOP_LIMIT_FRONT) ? F("PRESSED") : F("OPEN"));
  Serial.print(F(" TopBack="));
  Serial.print(isPressed(TOP_LIMIT_BACK) ? F("PRESSED") : F("OPEN"));
  Serial.print(F(" BottomFront="));
  Serial.print(isPressed(BOTTOM_LIMIT_FRONT) ? F("PRESSED") : F("OPEN"));
  Serial.print(F(" BottomBack="));
  Serial.println(isPressed(BOTTOM_LIMIT_BACK) ? F("PRESSED") : F("OPEN"));
}

static void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  if (command.length() == 0) return;

  if (command == "HOME") {
    startHome();
  } else if (command.startsWith("BPWM")) {
    startBottomPwmTest(command.substring(4).toInt());
  } else if (command.startsWith("TPWM")) {
    startTopPwmTest(command.substring(4).toInt());
  } else if (command.startsWith("B") && command.length() > 1) {
    commandBottomPosition(command.substring(1).toFloat());
  } else if (command.startsWith("T") && command.length() > 1) {
    commandTopPosition(command.substring(1).toFloat());
  } else if (command == "LIMITS" || command == "L") {
    printLimits();
  } else if (command == "STREAM ON") {
    limitStreamEnabled = true;
    Serial.println(F("[LIMITS] Stream ON"));
  } else if (command == "STREAM OFF") {
    limitStreamEnabled = false;
    Serial.println(F("[LIMITS] Stream OFF"));
  } else if (command == "STOP" || command == "X") {
    stopAll();
  } else if (command == "STATUS" || command == "?") {
    printStatus();
  } else if (command == "CLEAR") {
    bottomMotor(0);
    topMotor(0);
    state = IDLE;
    Serial.println(F("[FAULT] Cleared"));
  } else if (command == "HELP") {
    printHelp();
  } else {
    Serial.println(F("[ERROR] Unknown command. Send HELP."));
  }
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialLine.length() > 0) {
        processCommand(serialLine);
        serialLine = "";
      }
    } else if (serialLine.length() < 40) {
      serialLine += c;
    }
  }
}
