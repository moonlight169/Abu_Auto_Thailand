#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <Arduino.h>

// ============================================================
// arm_control.h — ฟังก์ชันควบคุม Motor + Encoder + PID + Calibrate
// Slave Arm — สไตล์ Code Kitty
// ============================================================

// ---------- Setup (เรียกครั้งเดียวใน setup()) ----------
void arm_init();

// ---------- Motor Control ----------
void arm_stop();                   // หยุดมอเตอร์ทั้งสองตัว

// ---------- อัปเดต (เรียกทุกครั้งใน loop()) ----------
void arm_update();    // เรียกทุก loop — จัดการ PID + safety cutoff

// ===================== BOTTOM (armDown) =====================

// Calibrate: หมุนชน limit → ถอยกลับ → reset count = 0°
void bottom_calibrate();

// PID หมุนไปที่ angle (0 - 180°)
void bottom_goTo(float deg);

// อ่านค่ามุมปัจจุบัน (0-180°)
float bottom_getAngle();

// สถานะ
bool bottom_isCalibrated();
bool bottom_isBusy();
bool bottom_atTarget();

// รีเซ็ต encoder + สถานะ (สำหรับ calibrate ใหม่)
void bottom_resetCalibrate();

// ===================== TOP (armUp) — Offset System =====================
// 0° = center (กึ่งกลางระหว่าง limit ทั้งสองฝั่ง)
// 90° = ไปทาง back limit, -90° = ไปทาง front limit

void top_goTo(float deg);       // 0=center, 90=back, -90=front
float top_getAngle();           // มุมเทียบกับ center
bool top_isCalibrated();
bool top_isBusy();
bool top_atTarget();

// ===================== SAFE COMMANDS =====================
// คำสั่ง: 0=bottom_0, 1=bottom_180, 2=top_0(center), 3=top_90, 4=top_-90, 9=bottom calibrate
// คืน true เมื่อรับคำสั่งไปทำแล้ว / false เมื่อโดนเงื่อนไข safety บล็อกหรือรหัสไม่รู้จัก
bool arm_sendCommand(int cmd);

#endif