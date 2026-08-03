// ============================================================
// 6_slave_f103 - STM32F103C8T6 (Blue Pill) Slave Board
// Minimal keypad echo: กดปุ่ม -> OLED โชว์ label -> ส่ง text ไป Teensy
// ============================================================
//
// Pin Assignment:
//   Serial1 (ต่อตรงไป Teensy Serial3, TX3=pin14/RX3=pin15): PA9=TX, PA10=RX
//   OLED (Software I2C): PB10=SCL, PB11=SDA
//   Keypad: PA2=C1, PA3=C2, PA0=C3, PA1=C4
//           PB0=R1, PB1=R2, PA6=R3, PA7=R4
//   Onboard LED: PC13
//
// Communication (plain text, ไม่มี framing/checksum):
//   กดปุ่ม 1 ครั้ง -> ส่ง label + '\n' ออก Serial1 ซ้ำ 5 ครั้ง ห่างกัน 20ms เช่น "A4\n"
//   กดค้างไม่ส่งซ้ำ ปล่อยปุ่มไม่ส่งอะไร
//   ฝั่ง Teensy อ่านเป็นบรรทัดได้เลย (Serial3.readStringUntil('\n'))
//   *** Teensy ต้องกรองซ้ำเอง — จะได้ "A4" 5 บรรทัดติดจากการกด 1 ครั้ง ***
// ============================================================

#include <Arduino.h>
#include "config.h"
#include "forest_menu.h"

void setup() {
    Serial.begin(115200);

    // Init keypad + OLED
    forest_menu_init();

    // Init Serial1 directly to Teensy Serial3
    // *** ฝั่ง Teensy ยังไม่มีโค้ดอ่านลิงก์นี้ — ตอนนี้ส่งออกไปลอย ๆ ***
    Serial1.begin(F103_SERIAL_BAUD);

    delay(200);
    Serial.println(F("=== F103 Keypad Echo ==="));
    Serial.println(F("Press key -> show A1-A8/B1-B8 on OLED -> send text to Teensy"));
}

// ===== Repeat-send state =====
// กด 1 ครั้ง -> ส่งซ้ำ REPEAT_COUNT ครั้ง ห่างกัน REPEAT_INTERVAL_MS
// เผื่อฝั่ง Teensy พลาดรอบแรก (buffer เต็ม / ยังไม่ทัน sync)
#define REPEAT_COUNT        5
#define REPEAT_INTERVAL_MS  20

static char          _txFrame[8] = "";   // "A4\n" ที่กำลังส่งอยู่
static uint8_t       _txFrameLen = 0;
static uint8_t       _txLeft = 0;        // เหลือต้องส่งอีกกี่ครั้ง
static unsigned long _txNextAt = 0;

void loop() {
    unsigned long now = millis();

    // 1. Update keypad scan + OLED display
    forest_menu_update(now);

    // 2. กดปุ่ม -> ตั้งคิวส่งซ้ำ (ปุ่มใหม่ยกเลิกคิวเก่าทิ้ง)
    if (forest_menu_wasPressed()) {
        const char* label = forest_menu_getLabel();

        _txFrameLen = (uint8_t)snprintf(_txFrame, sizeof(_txFrame), "%s\n", label);
        _txLeft = REPEAT_COUNT;
        _txNextAt = now;   // ส่งครั้งแรกทันทีในรอบนี้

        Serial.print(F("[KEY] "));
        Serial.print(label);
        Serial.print(F(" x"));
        Serial.println(REPEAT_COUNT);
    }

    // 3. ทยอยส่งตามคิว — non-blocking ไม่ delay() ขวางการสแกนปุ่ม
    if (_txLeft > 0 && (long)(now - _txNextAt) >= 0) {
        Serial1.write((const uint8_t*)_txFrame, _txFrameLen);
        _txLeft--;
        _txNextAt = now + REPEAT_INTERVAL_MS;
    }
}
