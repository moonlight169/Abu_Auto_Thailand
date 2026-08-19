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

    // Init keypad + OLED state machine
    forest_menu_init();

    // Init Serial1 -> Master Serial3 (baud ต้องแมตช์ BUTTON_BAUD = 115200)
    Serial1.begin(F103_SERIAL_BAUD);

    // ไม่มี delay() ตามกฎเหล็กข้อ 4 — U8g2.begin() ใน forest_menu_init()
    // จัดการ timing การ init จอเองแล้ว ไม่ต้องหน่วงเพิ่ม
    Serial.println(F("=== F103 Mission Keypad ==="));
    Serial.println(F("Enter code via keypad, press 16=START to send on Serial1."));
    Serial.println(F("This board takes no console commands; it only reports codes."));
}

// ===== Repeat-send state =====
// กด START 1 ครั้ง -> ส่งซ้ำ REPEAT_COUNT ครั้ง ห่างกัน REPEAT_INTERVAL_MS
// เผื่อฝั่ง Master พลาดรอบแรก (buffer เต็ม / ยังไม่ทัน sync) และให้ vote ทำงาน
//
// เฟรมแรกออกทันที ที่เหลืออีก 4 เฟรมห่างกัน 20ms -> ชุดหนึ่งกินเวลา 80ms
// สั้นกว่า BUTTON_VOTE_WINDOW_MS = 150ms ฝั่ง Master ทั้ง 5 เฟรมจึงตกในโหวตเดียวกัน
// และหน้า SENT ค้าง 900ms ก่อนจะรับรหัสใหม่ได้ -> โหวตรอบก่อนปิดไปแล้วเสมอ
// สองรหัสจึงไม่มีทางปนกันในหน้าต่างโหวตเดียว
#define REPEAT_COUNT        5
#define REPEAT_INTERVAL_MS  20

// รหัสยาวสุด 7 หลัก + '\n' + '\0' = 9 ไบต์ -> ใช้ 10 เผื่อกัน buffer overflow เด็ดขาด
static char          _txFrame[10] = "";
static uint8_t       _txFrameLen = 0;
static uint8_t       _txLeft = 0;        // เหลือต้องส่งอีกกี่ครั้ง
static unsigned long _txNextAt = 0;

void loop() {
    unsigned long now = millis();

    // 1. เดิน state machine (สแกนปุ่ม + อัปเดต OLED)
    forest_menu_update(now);

    // 2. กด START -> ตั้งคิวส่งรหัสซ้ำ (ชุดใหม่ยกเลิกคิวเก่าทิ้ง)
    if (forest_menu_hasCodeToSend()) {
        const char* code = forest_menu_getCode();

        // snprintf คืนความยาว "ก่อนถูกตัด" ไม่ใช่ที่เขียนจริง ถ้าวันไหนรหัสยาวเกินคาด
        // ค่าที่ได้จะเกินบัฟเฟอร์ แล้ว write() ข้างล่างจะอ่านเลยท้ายอาร์เรย์ -> clamp ไว้ก่อน
        int frameLen = snprintf(_txFrame, sizeof(_txFrame), "%s\n", code);
        if (frameLen < 0) frameLen = 0;
        if (frameLen > (int)sizeof(_txFrame) - 1) frameLen = (int)sizeof(_txFrame) - 1;

        _txFrameLen = (uint8_t)frameLen;
        _txLeft = REPEAT_COUNT;
        _txNextAt = now;   // ส่งครั้งแรกทันทีในรอบนี้

        Serial.print(F("[SEND] "));
        Serial.print(code);
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
