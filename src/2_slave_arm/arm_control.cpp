#include "arm_control.h"
#include "config_arm.h"
#include "motor.h"
#include "encoder.h"
#include <PID.h>

// ===================== CONSTANTS =====================

const int BOTTOM_CALIB_SPEED = -80;
const int BOTTOM_BACKOFF_SPEED = 40;
const int BOTTOM_BACKOFF_MS = 300;

const float PULSE_PER_DEG_BOTTOM = 890.0;
const float PULSE_PER_DEG_TOP    = -890.0;   // ลบ = กลับทิศทาง

const float PID_MIN = -254;
const float PID_MAX = 255;
const float PID_KP = 1.0;
const float PID_KI = 0.0;
const float PID_KD = 0.0;

const unsigned long CALIB_TIMEOUT_MS = 10000;

// ===================== STATIC OBJECTS =====================

static Motor _armUp(arm_upA, arm_upB);
static Motor _armDown(arm_downA, arm_downB);

static Encoder _encUp(enc_arm_upA, enc_arm_upB, 1.0);
static Encoder _encDown(enc_arm_downA, enc_arm_downB, 1.0);

static PID _pidBottom(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);
static PID _pidTop(PID_MIN, PID_MAX, PID_KP, PID_KI, PID_KD);

// ===================== STATE =====================

enum BottomState {
    BOTTOM_IDLE,
    BOTTOM_CALIBRATING,     // วิ่งหา limit switch
    BOTTOM_CALIB_PAUSE,     // เจอ switch แล้ว หยุดนิ่งรอก่อนถอย
    BOTTOM_CALIB_BACKOFF,   // ถอยออกจาก switch ก่อน reset ศูนย์
    BOTTOM_MOVING
};
enum TopState    { TOP_IDLE, TOP_MOVING };

static BottomState _bottomState = BOTTOM_IDLE;
static TopState _topState = TOP_IDLE;

static bool _bottomCalibrated = false;
static bool _topCalibrated = false;

static float _bottomTargetDeg = 0;
static float _topTargetDeg = 0;

static long _topCenter = 0;   // pulse count ที่ center

static unsigned long _calibStartTime = 0;
static unsigned long _calibStepTime = 0;   // จับเวลา pause/backoff ระหว่าง calibrate

// ===================== ISR =====================

static void _isrEncUpA()   { _encUp.handleA(); }
static void _isrEncUpB()   { _encUp.handleB(); }
static void _isrEncDownA() { _encDown.handleA(); }
static void _isrEncDownB() { _encDown.handleB(); }

// ===================== SAFETY =====================

static void _applySafety() {
    // อ่าน limit switch ทุกตัว (Active LOW = โดนกด)
    bool bottomFrontHit = (digitalRead(L_SW_arm_down_front_8) == LOW);
    bool bottomBackHit  = (digitalRead(L_SW_arm_down_back_9)  == LOW);
    bool topFrontHit    = (digitalRead(L_SW_arm_up_front_4)   == LOW);
    bool topBackHit     = (digitalRead(L_SW_arm_up_back_2)    == LOW);

    // ----- BOTTOM: speed < 0 = วิ่งเข้า front (0°), speed > 0 = วิ่งเข้า back (180°) -----
    bool bottomTowardFront = (_armDown.getSpeed() < 0);
    bool bottomTowardBack  = (_armDown.getSpeed() > 0);

    if (bottomFrontHit && bottomTowardFront) _armDown.run(0);
    if (bottomBackHit  && bottomTowardBack)  _armDown.run(0);

    // ----- TOP: speed < 0 = วิ่งเข้า front (-90°), speed > 0 = วิ่งเข้า back (+90°) -----
    // ยกเว้นเป้าหมายเป็น center (0°) ให้วิ่งได้เสมอ เพราะ center อยู่ระหว่าง limit สองฝั่งแน่นอน
    if (_topTargetDeg == 0) return;

    bool topTowardFront = (_armUp.getSpeed() < 0);
    bool topTowardBack  = (_armUp.getSpeed() > 0);

    if (topFrontHit && topTowardFront) _armUp.run(0);
    if (topBackHit  && topTowardBack)  _armUp.run(0);
}

// ===================== PUBLIC API =====================

void arm_init() {
    pinMode(L_SW_arm_up_front_4, INPUT_PULLUP);
    pinMode(L_SW_arm_up_back_2, INPUT_PULLUP);
    pinMode(L_SW_arm_down_front_8, INPUT_PULLUP);
    pinMode(L_SW_arm_down_back_9, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(enc_arm_upA), _isrEncUpA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_arm_upB), _isrEncUpB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_arm_downA), _isrEncDownA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(enc_arm_downB), _isrEncDownB, CHANGE);

    Serial.println("[ARM] init OK");
}

void arm_stop() {
    _armUp.run(0);
    _armDown.run(0);
    _bottomState = BOTTOM_IDLE;
    _topState = TOP_IDLE;
}

void arm_update() {
    unsigned long now = millis();

    // ===== BOTTOM =====
    if (_bottomState == BOTTOM_CALIBRATING) {
        if (digitalRead(L_SW_arm_down_front_8) == LOW || digitalRead(L_SW_arm_down_back_9) == LOW) {
            _armDown.run(0);
            _calibStepTime = now;
            _bottomState = BOTTOM_CALIB_PAUSE;
        }
        else if (now - _calibStartTime > CALIB_TIMEOUT_MS) {
            _armDown.run(0);
            _bottomState = BOTTOM_IDLE;
            Serial.println("[BOTTOM] Calibrate TIMEOUT");
        }
    }
    else if (_bottomState == BOTTOM_CALIB_PAUSE) {
        if (now - _calibStepTime >= BOTTOM_BACKOFF_MS) {
            _armDown.run(BOTTOM_BACKOFF_SPEED);
            _calibStepTime = now;
            _bottomState = BOTTOM_CALIB_BACKOFF;
        }
    }
    else if (_bottomState == BOTTOM_CALIB_BACKOFF) {
        if (now - _calibStepTime >= BOTTOM_BACKOFF_MS) {
            _armDown.run(0);
            _encDown.reset();
            _bottomCalibrated = true;
            _bottomState = BOTTOM_IDLE;
            Serial.println("[BOTTOM] Calibrate OK → 0°");
        }
    }
    else if (_bottomState == BOTTOM_MOVING) {
        bool hitLimit = false;
        if (_bottomTargetDeg > 90) {
            if (digitalRead(L_SW_arm_down_back_9) == LOW) {
                _armDown.run(0);
                _bottomState = BOTTOM_IDLE;
                hitLimit = true;
            }
        } else {
            if (digitalRead(L_SW_arm_down_front_8) == LOW) {
                _armDown.run(0);
                _bottomState = BOTTOM_IDLE;
                hitLimit = true;
            }
        }
        if (!hitLimit) {
            float target = _bottomTargetDeg * PULSE_PER_DEG_BOTTOM;
            float current = (float)_encDown.getCount();
            float output = _pidBottom.compute(target, current);
            _armDown.run((int)output);
        }
    }

    // ===== TOP =====
    if (_topState == TOP_MOVING) {
        float target = _topCenter + _topTargetDeg * PULSE_PER_DEG_TOP;
        float current = (float)_encUp.getCount();
        float output = _pidTop.compute(target, current);
        // Negate output because PULSE_PER_DEG_TOP is negative (inverted direction)
        _armUp.run(-(int)output);

        // Safety cutoff: allow movement to center (0°) even if at a limit
        // because center is always between both limits
        if (_topTargetDeg != 0) {
            if (output > 0 && digitalRead(L_SW_arm_up_front_4) == LOW) {
                _armUp.run(0);
                _topState = TOP_IDLE;
                Serial.println("[TOP] Hit FRONT limit");
            }
            if (output < 0 && digitalRead(L_SW_arm_up_back_2) == LOW) {
                _armUp.run(0);
                _topState = TOP_IDLE;
                Serial.println("[TOP] Hit BACK limit");
            }
        }
    }

    _applySafety();
}

// ===================== BOTTOM =====================

void bottom_calibrate() {
    _armDown.run(BOTTOM_CALIB_SPEED);
    _bottomState = BOTTOM_CALIBRATING;
    _calibStartTime = millis();
    Serial.println("[BOTTOM] Calibrating...");
}

void bottom_goTo(float deg) {
    if (!_bottomCalibrated) {
        Serial.println("[BOTTOM] Not calibrated!");
        return;
    }
    _bottomTargetDeg = constrain(deg, 0, 180);
    _bottomState = BOTTOM_MOVING;
    Serial.print("[BOTTOM] goTo ");
    Serial.println(_bottomTargetDeg);
}

float bottom_getAngle() {
    return (float)_encDown.getCount() / PULSE_PER_DEG_BOTTOM;
}

bool bottom_isCalibrated() { return _bottomCalibrated; }
bool bottom_isBusy()       { return _bottomState != BOTTOM_IDLE; }
bool bottom_atTarget() {
    float err = abs(_bottomTargetDeg * PULSE_PER_DEG_BOTTOM - _encDown.getCount());
    return err < 10.0;
}

void bottom_resetCalibrate() {
    _bottomCalibrated = false;
    _bottomState = BOTTOM_IDLE;
    _encDown.reset();
}

// ===================== TOP =====================

void top_goTo(float deg) {
    _topTargetDeg = deg;
    _topState = TOP_MOVING;
    Serial.print("[TOP] goTo ");
    Serial.println(deg);
}

float top_getAngle() {
    if (_topCenter == 0) return (float)_encUp.getCount() / PULSE_PER_DEG_TOP;
    return (float)(_encUp.getCount() - _topCenter) / PULSE_PER_DEG_TOP;
}

bool top_isCalibrated() { return _topCalibrated; }
bool top_isBusy()       { return _topState != TOP_IDLE; }
bool top_atTarget() {
    float target = _topCenter + _topTargetDeg * PULSE_PER_DEG_TOP;
    float err = abs(target - _encUp.getCount());
    return err < 10.0;
}

// ===================== SAFE COMMANDS =====================

bool arm_sendCommand(int cmd) {
    // Safety: bottom can only move if top is near center (0°)
    bool topAtCenter = (abs(top_getAngle()) < 15);
    // Safety: top ±90° can only move if bottom is near correct position
    bool bottomAt0   = (abs(bottom_getAngle()) < 15);
    bool bottomAt180 = (abs(bottom_getAngle() - 180) < 15);

    switch (cmd) {
        case 0:
            if (!_bottomCalibrated) { Serial.println("[CMD] Bottom not calibrated!"); return false; }
            if (!topAtCenter) { Serial.println("[CMD] Top must be at 0° (center) to move bottom!"); return false; }
            bottom_goTo(0);
            return true;
        case 1:
            if (!_bottomCalibrated) { Serial.println("[CMD] Bottom not calibrated!"); return false; }
            if (!topAtCenter) { Serial.println("[CMD] Top must be at 0° (center) to move bottom!"); return false; }
            bottom_goTo(180);
            return true;
        case 2:
            top_goTo(0);
            return true;
        case 3:
            if (!bottomAt0) { Serial.println("[CMD] Bottom must be at 0° for top 90°!"); return false; }
            top_goTo(90);
            return true;
        case 4:
            if (!bottomAt180) { Serial.println("[CMD] Bottom must be at 180° for top -90°!"); return false; }
            top_goTo(-90);
            return true;
        case 9:
            bottom_calibrate();
            return true;
        default:
            Serial.print("[CMD] Unknown: ");
            Serial.println(cmd);
            return false;
    }
}