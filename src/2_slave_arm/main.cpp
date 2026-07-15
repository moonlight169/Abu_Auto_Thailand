#include <Arduino.h>
#include "arm_control.h"
#include "protocol.h"

ArmReceiver armReceiver;
unsigned long lastLogTime = 0;
unsigned long lastFbTime = 0;
const unsigned long FB_INTERVAL_MS = 100;

// รหัสท่าจาก master (master ส่งซ้ำทุก 50ms)
uint8_t activeCode = 0xFF;              // รหัสล่าสุดที่สั่งสำเร็จแล้ว (0xFF = ยังไม่เคยสั่ง)
unsigned long lastCodeTryTime = 0;
const unsigned long CODE_RETRY_MS = 500;  // ถ้าโดน interlock บล็อก ลองใหม่ทุก 500ms

// จัดการรหัสคำสั่งชุดเดียวกับเมนู debug
// ใช้ร่วมกันทั้งจาก USB Serial และจาก master (sendArm) ให้พฤติกรรมตรงกันทุก path
void handleArmCode(char code){
    switch (code){
        case '0' ... '4':
            arm_sendCommand(code - '0');
            break;
        case '9':
            bottom_calibrate();
            break;
        case 's':
            if (bottom_isBusy()) bottom_resetCalibrate();
            arm_stop();
            break;
    }
}

void setup(){
    Serial.begin(115200);
    arm_init();
    Serial1.begin(115200);

    Serial.println(F("=== Slave Arm Ready ==="));
    Serial.println(F("0 = bottom go to 0°"));
    Serial.println(F("1 = bottom go to 180°"));
    Serial.println(F("2 = top go to center (0°)"));
    Serial.println(F("3 = top go to 90°"));
    Serial.println(F("4 = top go to -90°"));
    Serial.println(F("s = stop all"));
    Serial.println(F("? = print status"));
}

void loop() {
    arm_update();

    // ===== 1. Receive protocol from master (Serial1) =====
    while (Serial1.available() > 0){
        uint8_t incomingByte = Serial1.read();
        armReceiverFeed(armReceiver, incomingByte);
    }

    if (armReceiver.hasNewArmCommand){
        ArmCommand &cmd = armReceiver.lastArmCommand;

        if (cmd.flags & 0x01) bottom_calibrate();
        if (cmd.flags & 0x04){
            if (bottom_isBusy()) bottom_resetCalibrate();
            arm_stop();
        }
        if (cmd.flags & 0x08) bottom_resetCalibrate();

        if (cmd.bottomAngle != 0xFF) bottom_goTo(cmd.bottomAngle);
        if (cmd.topAngle != 0xFF){
            // topAngle เป็น signed int8 จริงๆ (0-255 → map เป็น -128 ถึง 127)
            int8_t signedTop = (int8_t)cmd.topAngle;
            top_goTo((float)signedTop);
        }

        armReceiver.hasNewArmCommand = false;
    }

    // รหัสท่าจาก master (sendArmCommand) — เข้าเงื่อนไข safety ชุดเดียวกับเมนู debug
    // ทำงานเฉพาะรหัสที่ยังไม่สำเร็จ ถ้าโดน interlock บล็อกจะลองใหม่เองทุก 500ms
    if (armReceiver.hasNewArmCode){
        uint8_t code = armReceiver.lastArmCode;
        if (code != activeCode && millis() - lastCodeTryTime >= CODE_RETRY_MS){
            if (arm_sendCommand(code)){
                activeCode = code;
            }
            lastCodeTryTime = millis();
        }
        armReceiver.hasNewArmCode = false;
    }

    // ===== 2. Debug commands via USB Serial =====
    if (Serial.available()){
        int cmd = Serial.read();
        handleArmCode((char)cmd);
        Serial.flush();
    }

    // ===== 3. Send feedback to master =====
    if (millis() - lastFbTime >= FB_INTERVAL_MS){
        lastFbTime = millis();

        uint8_t fbBottom = (uint8_t)constrain((int)bottom_getAngle(), 0, 255);
        int topAngleInt = (int)top_getAngle();
        uint8_t fbTop = (uint8_t)(topAngleInt & 0xFF);

        uint8_t status = 0;
        if (bottom_isCalibrated()) status |= 0x01;
        if (bottom_isBusy())       status |= 0x02;
        if (bottom_atTarget())     status |= 0x04;
        if (top_isCalibrated())    status |= 0x08;
        if (top_isBusy())          status |= 0x10;
        if (top_atTarget())        status |= 0x20;

        sendArmFeedback(Serial1, fbBottom, fbTop, status);
    }

    // ===== 4. Print status every 500ms =====
    if (millis() - lastLogTime >= 500){
        lastLogTime = millis();
        Serial.print(F("Bottom: "));
        Serial.print(bottom_getAngle(), 1);
        Serial.print(F("°  Top: "));
        Serial.print(top_getAngle(), 1);
        Serial.print(F("°  Bcal:"));
        Serial.print(bottom_isCalibrated() ? 'y' : 'n');
        Serial.print(F("  Tcal:"));
        Serial.print(top_isCalibrated() ? 'y' : 'n');
        Serial.print(F("  Bbusy:"));
        Serial.print(bottom_isBusy() ? 'y' : 'n');
        Serial.print(F("  Tbusy:"));
        Serial.println(top_isBusy() ? 'y' : 'n');
    }
}