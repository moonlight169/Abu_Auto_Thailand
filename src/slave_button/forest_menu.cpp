// ============================================================
// forest_menu.cpp - Mission-code entry state machine (F103)
// ============================================================
// พิมพ์รหัส mission ผ่าน keypad 4x4 -> โชว์บน OLED -> main.cpp ส่งไป Master
//
// โครง state machine (ดูตารางปุ่มใน forest_menu.h):
//   PAGE_MODE   เลือกโหมด  (ปุ่ม 3 สลับ ป่า/ramp)          -> ENTER
//   PAGE_FIELD  เลือกสนาม  (ปุ่ม 8 BLUE / ปุ่ม 12 RED)      -> ENTER
//   PAGE_SEL3   ป่า:LINE | ramp:STEP                         -> ENTER
//   PAGE_SEL4   ป่า:BOX  | ramp:ROW                          -> ENTER
//   PAGE_RECHECK ทวนรหัสเต็ม แล้วกด START (ปุ่ม 16) ส่งจริง
//   PAGE_SENT   โชว์ "SENT" ~0.9s แล้วรีเซ็ตกลับหน้าแรกเอง (non-blocking)
//
// รูปแบบรหัส (ตรงกับ mission_list.cpp + MISSION_CODE_LENGTHS ฝั่ง Master):
//   ป่า  = 7 หลัก: [0][field][line][box1..4]
//   ramp = 5 หลัก: [1][field][step][pos1][pos2]
//     step=0 (All) -> pos1=row ของชั้น2, pos2=row ของชั้น3 (อิสระ/ซ้ำได้)
//     step=2/3     -> pos1=row, pos2=0 (เติมให้ตอน ENTER)
//
// ปรับปรุงจากเวอร์ชันก่อน (logic ปุ่ม/ส่ง คงเดิมทุกอย่าง):
//   1. OLED = full buffer (_F_) -> วาดครั้งเดียว/เฟรม ลื่น ไม่กระพริบ (เดิม _1_ วน 8 รอบ)
//   2. debounce 5 -> 18ms + ยืนยัน 2 รอบ -> กันปุ่มเด้ง/ghost บน matrix จริง
//   3. หน้า SENT + LED ยืนยันว่า "ส่งออกไปจริง" ไม่ใช่แค่โชว์บนจอ
//   4. topbar breadcrumb + hint ปุ่มล่างจอ -> ดูรู้เรื่อง ไม่ต้องจำปุ่ม
// ============================================================

#include "forest_menu.h"
#include "config.h"
#include <string.h>
#include <stdio.h>

// ===== OLED instance — full buffer (ลื่นกว่า/ไม่ tearing สำหรับ software I2C) =====
static U8G2_SH1106_128X64_NONAME_F_SW_I2C _u8g2(U8G2_R0, OLED_SOFT_SCL, OLED_SOFT_SDA, U8X8_PIN_NONE);

static const uint8_t _colPins[4] = { KEYPAD_C1, KEYPAD_C2, KEYPAD_C3, KEYPAD_C4 };
static const uint8_t _rowPins[4] = { KEYPAD_R1, KEYPAD_R2, KEYPAD_R3, KEYPAD_R4 };

// ===== Debounce state =====
static bool          _keyLastState[4][4] = { { false } };
static unsigned long _keyLastDebounce[4][4] = { { 0 } };
static bool          _keyState[4][4] = { { false } };
static bool          _keyPrevState[4][4] = { { false } };

// ค่าดิบต้องนิ่งเท่านี้ถึงนับเป็นการเปลี่ยนสถานะ — 18ms กันปุ่มเด้งบน matrix จริง
#define DEBOUNCE_MS 18

// ===== State machine =====
enum Page : uint8_t {
    PAGE_MODE = 0,
    PAGE_FIELD,
    PAGE_SEL3,     // ป่า: LINE | ramp: STEP
    PAGE_SEL4,     // ป่า: BOX  | ramp: ROW
    PAGE_RECHECK,
    PAGE_SENT      // โชว์ยืนยันหลังส่ง แล้วรีเซ็ตเอง
};

enum Mode : uint8_t {
    MODE_FOREST = 0,   // ขึ้นป่า -> รหัส 7 หลัก
    MODE_RAMP   = 1    // ramp    -> รหัส 5 หลัก
};

#define UNSET 0xFF
#define SENT_HOLD_MS 900   // โชว์หน้า SENT นานเท่านี้ก่อนรีเซ็ต (non-blocking)

static Page    _page    = PAGE_MODE;
static uint8_t _mode    = MODE_FOREST;
static uint8_t _field   = UNSET;         // 0=BLUE, 1=RED

// --- FOREST ---
static uint8_t _line    = 0;             // 1..3 (0=ยังไม่เลือก)
static char    _box[5]  = "";            // box 4 หลัก + '\0'
static uint8_t _boxLen  = 0;

// --- RAMP ---
static uint8_t _step    = UNSET;         // 0=All / 2=ชั้น2 / 3=ชั้น3
static uint8_t _rampRow1 = 0;
static uint8_t _rampRow2 = 0;
static uint8_t _rampRowCount = 0;

static char     _code[9]    = "";        // รหัสเต็มที่พร้อมส่ง
static bool     _codeReady  = false;     // one-shot: กด START แล้วรหัสพร้อม
static bool     _dirty      = true;      // ต้อง re-render OLED
static uint32_t _sentAt     = 0;         // เวลาเข้าหน้า SENT

// ============================================================
// Keypad scan + debounce
// ============================================================
static uint16_t _scanKeypad() {
    uint16_t rawMask = 0;
    for (int c = 0; c < 4; c++) {
        digitalWrite(_colPins[c], LOW);
        delayMicroseconds(3);   // settle สัญญาณ column (ไม่ใช่ blocking delay ของ loop)
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

// คืนเลขปุ่ม 1..16 ที่ "เพิ่งกด" (ขอบขาลง) — 0 = ไม่มีปุ่มใหม่
static uint8_t _consumePressedKey() {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            if (_keyState[r][c] && !_keyPrevState[r][c]) {
                return (uint8_t)(r * 4 + c + 1);
            }
        }
    }
    return 0;
}

// ============================================================
// State helpers  (logic คงเดิมทั้งหมด)
// ============================================================
static void _boxAppend(char bit) {
    if (_boxLen < 4) {
        _box[_boxLen++] = bit;
        _box[_boxLen] = '\0';
    } else {
        _box[0] = _box[1];
        _box[1] = _box[2];
        _box[2] = _box[3];
        _box[3] = bit;
    }
}

static void _rampRowPress(uint8_t rowVal) {
    if (_step == 0) {   // All: สะสม 2 ตำแหน่ง (pos1=ชั้น2, pos2=ชั้น3)
        if (_rampRowCount == 0) {
            _rampRow1 = rowVal;
            _rampRowCount = 1;
        } else if (_rampRowCount == 1) {
            _rampRow2 = rowVal;
            _rampRowCount = 2;
        } else {
            _rampRow1 = _rampRow2;   // ครบ 2 แล้ว -> เลื่อนเอาล่าสุด
            _rampRow2 = rowVal;
        }
    } else {            // ชั้น2/3: 1 ตำแหน่ง กดทับได้
        _rampRow1 = rowVal;
        _rampRowCount = 1;
    }
}

static void _buildCode() {
    if (_mode == MODE_FOREST) {
        snprintf(_code, sizeof(_code), "0%u%u%s",
                 (unsigned)_field, (unsigned)_line, _box);
    } else if (_step == 0) {
        snprintf(_code, sizeof(_code), "1%u0%u%u",
                 (unsigned)_field, (unsigned)_rampRow1, (unsigned)_rampRow2);
    } else {
        snprintf(_code, sizeof(_code), "1%u%u%u0",
                 (unsigned)_field, (unsigned)_step, (unsigned)_rampRow1);
    }
}

static void _resetSelection() {
    _page    = PAGE_MODE;
    _mode    = MODE_FOREST;
    _field   = UNSET;
    _line    = 0;
    _box[0]  = '\0';
    _boxLen  = 0;
    _step    = UNSET;
    _rampRow1 = 0;
    _rampRow2 = 0;
    _rampRowCount = 0;
}

static bool _pageReadyToAdvance() {
    switch (_page) {
        case PAGE_MODE:  return true;
        case PAGE_FIELD: return _field != UNSET;
        case PAGE_SEL3:
            if (_mode == MODE_FOREST) return _line != 0;
            else                      return _step != UNSET;
        case PAGE_SEL4:
            if (_mode == MODE_FOREST) {
                return _boxLen == 4;
            } else {
                if (_step == 0) return _rampRowCount == 2;
                else            return _rampRow1 != 0;
            }
        default:         return false;
    }
}

// ============================================================
// จัดการปุ่มตามหน้าปัจจุบัน  (mapping คงเดิมทั้งหมด)
// ============================================================
static void _handleKey(uint8_t key) {
    // ---- ENTER (ปุ่ม 4) ----
    if (key == 4) {
        if (!_pageReadyToAdvance())
            return;
        if (_page == PAGE_SEL4) {
            _buildCode();
            _page = PAGE_RECHECK;
        } else if (_page < PAGE_RECHECK) {
            _page = (Page)(_page + 1);
        }
        return;
    }

    // ---- START (ปุ่ม 16): ยิงรหัสจริง เฉพาะหน้า RECHECK ----
    if (key == 16) {
        if (_page == PAGE_RECHECK) {
            _codeReady = true;               // main.cpp จะอ่าน getCode() แล้วส่ง
            digitalWrite(F103_LED, LOW);     // LED ติด = ยืนยันส่ง (active LOW)
            _page = PAGE_SENT;               // โชว์ SENT (ยังไม่ล้างค่า เพื่อให้ main อ่าน _code ได้)
            _sentAt = millis();
        }
        return;
    }

    // ---- ปุ่มเลือกค่าเฉพาะหน้า ----
    switch (_page) {
        case PAGE_MODE:
            if (key == 3) {
                _mode = (_mode == MODE_FOREST) ? MODE_RAMP : MODE_FOREST;
                _line = 0;
                _step = UNSET;
                _rampRow1 = _rampRow2 = 0;
                _rampRowCount = 0;
                _box[0] = '\0';
                _boxLen = 0;
            }
            break;

        case PAGE_FIELD:
            if (key == 8)  _field = 0;
            if (key == 12) _field = 1;
            break;

        case PAGE_SEL3:
            if (_mode == MODE_FOREST) {
                if (key == 15) _line = 1;
                if (key == 11) _line = 2;
                if (key == 7)  _line = 3;
            } else {
                if (key == 13) _step = 0;
                if (key == 9)  _step = 2;
                if (key == 5)  _step = 3;
            }
            break;

        case PAGE_SEL4:
            if (_mode == MODE_FOREST) {
                if (key == 1) _boxAppend('0');
                if (key == 2) _boxAppend('1');
            } else {
                if (key == 14) _rampRowPress(1);
                if (key == 10) _rampRowPress(2);
                if (key == 6)  _rampRowPress(3);
            }
            break;

        default:
            break;
    }
}

// ============================================================
// OLED render — full buffer, layout: topbar / content / hint
// ============================================================
static const char* _fieldText() {
    if (_field == 0) return "BLUE";
    if (_field == 1) return "RED";
    return "-";
}

static const char* _stepText() {
    if (_step == 0) return "All";
    if (_step == 2) return "Lv2";
    if (_step == 3) return "Lv3";
    return "-";
}

// breadcrumb: ค่าที่เลือกไปแล้ว เช่น "RAMP RED All"
static void _buildBreadcrumb(char* out, size_t n) {
    out[0] = '\0';
    if (_page > PAGE_MODE) {
        strncat(out, _mode == MODE_FOREST ? "FRST" : "RAMP", n - strlen(out) - 1);
    }
    if (_page > PAGE_FIELD && _field != UNSET) {
        strncat(out, " ", n - strlen(out) - 1);
        strncat(out, _fieldText(), n - strlen(out) - 1);
    }
    if (_page > PAGE_SEL3) {
        strncat(out, " ", n - strlen(out) - 1);
        if (_mode == MODE_FOREST) {
            char t[4]; snprintf(t, sizeof(t), "L%u", (unsigned)_line);
            strncat(out, t, n - strlen(out) - 1);
        } else {
            strncat(out, _stepText(), n - strlen(out) - 1);
        }
    }
}

// แถบบน: breadcrumb ซ้าย + step number ขวา + เส้นคั่น
static void _drawTopBar(uint8_t stepNum) {
    char bc[24];
    _buildBreadcrumb(bc, sizeof(bc));

    _u8g2.setFont(u8g2_font_5x8_tr);
    _u8g2.drawStr(0, 7, bc);

    char sn[8];
    snprintf(sn, sizeof(sn), "%u/5", (unsigned)stepNum);
    int w = _u8g2.getStrWidth(sn);
    _u8g2.drawStr(128 - w, 7, sn);

    _u8g2.drawHLine(0, 10, 128);
}

// หัวข้อหน้า (ใต้ topbar)
static void _drawTitle(const char* title) {
    _u8g2.setFont(u8g2_font_6x12_tr);
    _u8g2.drawStr(0, 23, title);
}

// ค่าใหญ่กลางจอ
static void _drawBigCentered(const char* text, int baselineY) {
    _u8g2.setFont(u8g2_font_logisoso20_tr);
    int w = _u8g2.getStrWidth(text);
    _u8g2.drawStr((128 - w) / 2, baselineY, text);
}

// hint ปุ่มล่างจอ
static void _drawHint(const char* hint) {
    _u8g2.setFont(u8g2_font_5x8_tr);
    _u8g2.drawStr(0, 63, hint);
}

static void _renderOled() {
    char line[24];

    _u8g2.clearBuffer();

    switch (_page) {
        case PAGE_MODE:
            _drawTopBar(1);
            _drawTitle("MODE");
            _drawBigCentered(_mode == MODE_FOREST ? "FOREST" : "RAMP", 48);
            _drawHint("3=switch   4=NEXT");
            break;

        case PAGE_FIELD:
            _drawTopBar(2);
            _drawTitle("FIELD");
            _drawBigCentered(_fieldText(), 48);
            _drawHint("8=BLUE 12=RED  4=NEXT");
            break;

        case PAGE_SEL3:
            if (_mode == MODE_FOREST) {
                _drawTopBar(3);
                _drawTitle("LINE");
                if (_line == 0) _drawBigCentered("-", 48);
                else { snprintf(line, sizeof(line), "%u", (unsigned)_line); _drawBigCentered(line, 48); }
                _drawHint("15/11/7=1/2/3  4=NEXT");
            } else {
                _drawTopBar(3);
                _drawTitle("STEP");
                _drawBigCentered(_stepText(), 48);
                _drawHint("13=All 9=L2 5=L3  4=NEXT");
            }
            break;

        case PAGE_SEL4:
            if (_mode == MODE_FOREST) {
                _drawTopBar(4);
                _drawTitle("BOX");
                _drawBigCentered(_boxLen ? _box : "----", 48);
                _drawHint("1=0  2=1  4=NEXT");
            } else if (_step == 0) {
                _drawTopBar(4);
                _drawTitle("ROW  L2,L3");
                char r1 = _rampRowCount >= 1 ? ('0' + _rampRow1) : '_';
                char r2 = _rampRowCount >= 2 ? ('0' + _rampRow2) : '_';
                snprintf(line, sizeof(line), "%c %c", r1, r2);
                _drawBigCentered(line, 48);
                _drawHint("14/10/6=Row (x2)  4=NEXT");
            } else {
                _drawTopBar(4);
                _drawTitle("ROW");
                if (_rampRow1 == 0) _drawBigCentered("-", 48);
                else { snprintf(line, sizeof(line), "%u", (unsigned)_rampRow1); _drawBigCentered(line, 48); }
                _drawHint("14/10/6=1/2/3  4=NEXT");
            }
            break;

        case PAGE_RECHECK:
            _buildCode();   // ประกอบสดให้ทวน
            _drawTopBar(5);
            // รหัสตัวใหญ่มีกรอบ
            {
                _u8g2.setFont(u8g2_font_logisoso20_tr);
                int w = _u8g2.getStrWidth(_code);
                int x = (128 - w) / 2;
                _u8g2.drawStr(x, 42, _code);
                _u8g2.drawFrame(x - 6, 20, w + 12, 28);
            }
            _drawHint(">> Press 16 = START");
            break;

        case PAGE_SENT:
            _u8g2.setFont(u8g2_font_logisoso24_tr);
            {
                const char* s = "SENT";
                int w = _u8g2.getStrWidth(s);
                _u8g2.drawStr((128 - w) / 2, 34, s);
            }
            _u8g2.setFont(u8g2_font_6x12_tr);
            {
                int w = _u8g2.getStrWidth(_code);
                _u8g2.drawStr((128 - w) / 2, 54, _code);
            }
            break;
    }

    _u8g2.sendBuffer();
}

// ============================================================
// PUBLIC API
// ============================================================
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

    pinMode(F103_LED, OUTPUT);
    digitalWrite(F103_LED, HIGH);   // LED ดับ (active LOW)

    _resetSelection();
    _codeReady = false;
    _dirty = true;

    Serial.println(F("[KEY] Mission keypad init OK"));
}

void forest_menu_update(unsigned long now) {
    (void)now;

    // ออกจากหน้า SENT อัตโนมัติเมื่อครบเวลา (non-blocking) แล้วรีเซ็ต
    if (_page == PAGE_SENT && (millis() - _sentAt) >= SENT_HOLD_MS) {
        digitalWrite(F103_LED, HIGH);   // LED ดับ
        _resetSelection();
        _dirty = true;
    }

    _updateDebounce();

    uint8_t key = _consumePressedKey();
    if (key != 0) {
        _handleKey(key);
        _dirty = true;

        Serial.print(F("[KEY] #"));
        Serial.print(key);
        Serial.print(F(" page="));
        Serial.println((int)_page);
    }

    if (_dirty) {
        _renderOled();
        _dirty = false;
    }
}

bool forest_menu_hasCodeToSend() {
    if (_codeReady) {
        _codeReady = false;
        return true;
    }
    return false;
}

const char* forest_menu_getCode() {
    return _code;
}
