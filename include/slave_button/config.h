#pragma once

#include <Arduino.h>

// ===== Pin Definitions for STM32F103 Slave (6_slave_f103) =====
//
// STM32F103C8T6 (Blue Pill) pin map:
// - PA9  = USART1_TX  (→ Teensy Serial3 RX, pin15)
// - PA10 = USART1_RX  (← Teensy Serial3 TX, pin14)
// - PA11 = USB_D-
// - PA12 = USB_D+
// - PA13 = SWDIO (ST-Link)
// - PA14 = SWCLK (ST-Link)
//
// Direct wiring to Teensy 4.1 hardware Serial3 (no bridge board):
//   Teensy pin 15 (RX3) ← F103 PA9  (TX)
//   Teensy pin 14 (TX3) → F103 PA10 (RX)
//   GND → GND
//
// ===== Serial to Teensy =====
// F103 uses hardware Serial1 (PA9=TX, PA10=RX)
// ต้องแมตช์ Master Serial3 (BUTTON_BAUD = 115200) — ฝั่ง master อ่านลิงก์นี้แล้ว
// ผ่าน readButtonSerial() -> vote 150ms -> startMissionByCode()
#define F103_SERIAL_BAUD 115200

// ===== OLED 1.3" SH1106 128x64 (Software I2C using U8g2) =====
#define OLED_SOFT_SCL PA8
#define OLED_SOFT_SDA PB14

// ===== 4x4 Matrix Keypad (ZX-SW16) =====
// IDC 8-pin header:
//   Pin 1 (C1) -> PA2    Pin 5 (R1) -> PB0
//   Pin 2 (C2) -> PA3    Pin 6 (R2) -> PB1
//   Pin 3 (C3) -> PA0    Pin 7 (R3) -> PA6
//   Pin 4 (C4) -> PA1    Pin 8 (R4) -> PA7
#define KEYPAD_C1 PA2
#define KEYPAD_C2 PA3
#define KEYPAD_C3 PA0
#define KEYPAD_C4 PA1
#define KEYPAD_R1 PB0
#define KEYPAD_R2 PB1
#define KEYPAD_R3 PA6
#define KEYPAD_R4 PA7

#define F103_LED PC13  // Onboard LED (Active LOW)
#define F103_SLAVE_ID 6