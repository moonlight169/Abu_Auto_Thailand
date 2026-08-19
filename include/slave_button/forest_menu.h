#ifndef FOREST_MENU_H
#define FOREST_MENU_H

#include <Arduino.h>
#include <U8g2lib.h>

// ============================================================
// Mission-code entry menu (state machine) สำหรับ F103 keypad panel
// ============================================================
// เดินหน้าจอ OLED 5 หน้า:
//   MODE -> FIELD -> SEL3 -> SEL4 -> RECHECK
//     SEL3 = ป่า:LINE  | ramp:STEP
//     SEL4 = ป่า:BOX   | ramp:ROW
// ผู้ใช้พิมพ์ผ่าน keypad 4x4 แล้วกด START ที่หน้า RECHECK
// -> main.cpp ส่งรหัส mission เป็น ASCII ตัวเลขล้วน + '\n' ออก Serial1 ไป Master
//
// รูปแบบรหัส (ตรงกับ mission_list.cpp + MISSION_CODE_LENGTHS ฝั่ง Master):
//   Mode 0 (ป่า)  = 7 หลัก: [0][field][line][box1..4]
//   Mode 1 (ramp) = 5 หลัก: [1][field][step][pos1][pos2]
//     field : 0=BLUE, 1=RED
//     line/row : 1..3
//     box  : แต่ละหลัก 0=ไม่เอากล่อง / 1=เอากล่อง
//     step : 0=All, 2=ชั้น2, 3=ชั้น3
//       step=0 (All) -> pos1=row ของชั้น2, pos2=row ของชั้น3 (อิสระ/ซ้ำได้)
//       step=2/3     -> pos1=row, pos2=0 (เติมให้ตอน ENTER)
//   ตัวอย่าง: 0120011 (ป่า/แดง/line2/box0,0,1,1)
//             11023 (ramp/แดง/All/ชั้น2@row2/ชั้น3@row3) | 11210 (ramp/แดง/ชั้น2@row1)
//
// keypad นับปุ่ม 1..16 (row-major): key = row*4 + col + 1
//   [ 1][ 2][ 3][ 4]      1=box0  2=box1  3=modeToggle  4=ENTER
//   [ 5][ 6][ 7][ 8]      5=step3 6=row3  7=line3       8=BLUE
//   [ 9][10][11][12]      9=step2 10=row2 11=line2      12=RED
//   [13][14][15][16]      13=All  14=row1 15=line1      16=START
// ============================================================

void forest_menu_init();
void forest_menu_update(unsigned long now);

// true หนึ่งครั้งเมื่อผู้ใช้กด START และรหัสพร้อมส่ง (one-shot: อ่านแล้วเคลียร์)
bool forest_menu_hasCodeToSend();

// รหัสเต็มที่พร้อมส่ง เช่น "0120011" — ใช้ได้ทันทีหลัง forest_menu_hasCodeToSend() คืน true
const char* forest_menu_getCode();

#endif