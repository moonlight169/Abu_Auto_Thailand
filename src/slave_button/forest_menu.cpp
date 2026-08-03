// ============================================================
// forest_menu.cpp - Minimal keypad echo for F103
// ============================================================
// กดปุ่ม -> OLED โชว์ label 2 วิ แล้วกลับเป็น "-" -> main.cpp ส่ง label เป็น text ไป Teensy
// ไม่มี state machine, ไม่มีเมนู — เอาแค่ให้เห็นว่าปุ่มไหนถูกกด
//
// Key number = row*4 + col + 1  ->  label:
//   R1:  1  2  3  4   ->  A1 A2 A3 A4
//   R2:  5  6  7  8   ->  A5 A6 A7 A8
//   R3:  9 10 11 12   ->  B1 B2 B3 B4
//   R4: 13 14 15 16   ->  B5 B6 B7 B8
// ============================================================

#include "forest_menu.h"
#include "config.h"
#include <string.h>

// ===== OLED instance (Software I2C) =====
static U8G2_SH1106_128X64_NONAME_1_SW_I2C _u8g2(U8G2_R0, OLED_SOFT_SCL, OLED_SOFT_SDA, U8X8_PIN_NONE);

static const uint8_t _colPins[4] = { KEYPAD_C1, KEYPAD_C2, KEYPAD_C3, KEYPAD_C4 };
static const uint8_t _rowPins[4] = { KEYPAD_R1, KEYPAD_R2, KEYPAD_R3, KEYPAD_R4 };

// ===== Debounce state =====
static bool _keyLastState[4][4] = {false};
static unsigned long _keyLastDebounce[4][4] = {0};
static bool _keyState[4][4] = {false};
static bool _keyPrevState[4][4] = {false};

// ===== Event state =====
static char _label[4] = "-";   // label ปุ่มล่าสุด เช่น "A1", "B8" ("-" = ว่าง)
static bool _wasPressed = false;
static bool _dirty = true;
static unsigned long _labelShownAt = 0;   // เวลาที่เริ่มโชว์ label ปัจจุบัน

// จอค้าง label ไว้เท่านี้แล้วกลับเป็น "-" — กันสับสนว่า label เก่าคือปุ่มที่เพิ่งกด
#define LABEL_HOLD_MS 2000

// ค่าดิบต้องนิ่งเท่านี้ถึงนับเป็นการเปลี่ยนสถานะ — กันปุ่มเด้ง
#define DEBOUNCE_MS 5

// ===== Keypad scan =====
static uint16_t _scanKeypad() {
    uint16_t rawMask = 0;
    for (int c = 0; c < 4; c++) {
        digitalWrite(_colPins[c], LOW);
        delayMicroseconds(2);
        for (int r = 0; r < 4; r++) {
            if (digitalRead(_rowPins[r]) == LOW)
                rawMask |= (1 << (r * 4 + c));
        }
        digitalWrite(_colPins[c], HIGH);
    }
    return rawMask;
}

static void _updateDebounce() {
    memcpy(_keyPrevState, _keyState, sizeof(_keyState));
    uint16_t rawMask = _scanKeypad();
    unsigned long now = millis();

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            bool raw = (rawMask & (1 << (r * 4 + c))) != 0;
            if (raw != _keyLastState[r][c]) {
                _keyLastDebounce[r][c] = now;
                _keyLastState[r][c] = raw;
            }
            if ((now - _keyLastDebounce[r][c]) >= DEBOUNCE_MS) {
                _keyState[r][c] = raw;
            }
        }
    }
}

// ===== Key number (1-16) -> label "A1".."A8" / "B1".."B8" =====
static void _makeLabel(uint8_t key, char* out) {
    if (key >= 1 && key <= 8) {
        out[0] = 'A';
        out[1] = '0' + key;
    } else {
        out[0] = 'B';
        out[1] = '0' + (key - 8);
    }
    out[2] = '\0';
}

// ===== OLED render =====
// โชว์ label ตัวใหญ่กลางจอ
static void _renderOled() {
    _u8g2.firstPage();
    do {
        _u8g2.setFont(u8g2_font_logisoso32_tr);
        int w = _u8g2.getStrWidth(_label);
        _u8g2.drawStr((128 - w) / 2, 48, _label);
    } while (_u8g2.nextPage());
}

// ===== PUBLIC API =====

void forest_menu_init() {
    _u8g2.begin();
    _u8g2.setContrast(128);

    for (int i = 0; i < 4; i++) {
        pinMode(_colPins[i], OUTPUT);
        digitalWrite(_colPins[i], HIGH);
    }
    for (int i = 0; i < 4; i++) {
        pinMode(_rowPins[i], INPUT_PULLUP);
    }

    strcpy(_label, "-");
    _labelShownAt = 0;
    _wasPressed = false;
    _dirty = true;

    Serial.println(F("[KEY] Keypad echo init OK"));
}

void forest_menu_update(unsigned long now) {
    _updateDebounce();

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            // ขอบขาลงเท่านั้น — ปล่อยปุ่มไม่ส่งอะไร
            if (_keyState[r][c] && !_keyPrevState[r][c]) {
                _makeLabel(r * 4 + c + 1, _label);
                _labelShownAt = now;
                _wasPressed = true;
                _dirty = true;
            }
        }
    }

    // โชว์ครบ LABEL_HOLD_MS แล้วเคลียร์จอกลับเป็น "-"
    if (_label[0] != '-' && (now - _labelShownAt) >= LABEL_HOLD_MS) {
        strcpy(_label, "-");
        _dirty = true;
    }

    if (_dirty) {
        _renderOled();
        _dirty = false;
    }
}

bool forest_menu_wasPressed() {
    if (_wasPressed) {
        _wasPressed = false;
        return true;
    }
    return false;
}

const char* forest_menu_getLabel() {
    return _label;
}
