#ifndef FOREST_MENU_H
#define FOREST_MENU_H

#include <Arduino.h>
#include <U8g2lib.h>

// ===== Minimal keypad echo =====
// กดปุ่ม -> OLED โชว์ label -> ผู้เรียกส่ง label เป็น text ไป Teensy
//
// Key number = row*4 + col + 1  ->  label:
//   R1:  1  2  3  4   ->  A1 A2 A3 A4
//   R2:  5  6  7  8   ->  A5 A6 A7 A8
//   R3:  9 10 11 12   ->  B1 B2 B3 B4
//   R4: 13 14 15 16   ->  B5 B6 B7 B8

void forest_menu_init();
void forest_menu_update(unsigned long now);

// Query: มีปุ่มถูกกดหรือไม่ (one-shot, อ่านแล้วเคลียร์) — ขอบขาลงเท่านั้น
// กด 1 ครั้ง = true 1 ครั้ง ต่อให้กดค้างก็ไม่ซ้ำ ปล่อยปุ่มก็ไม่นับ
bool forest_menu_wasPressed();

// Label ของปุ่มล่าสุด เช่น "A1", "B8" — คืน "-" หลังโชว์ครบ 2 วิ
// อ่านทันทีหลัง forest_menu_wasPressed() คืน true จะได้ label ที่เพิ่งกดเสมอ
const char* forest_menu_getLabel();

#endif
