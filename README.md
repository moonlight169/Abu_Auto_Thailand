# Abu_Auto_Thailand

โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์อัตโนมัติ (Auto Robot - R2)
ในการแข่งขัน ABU Robocon พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์
by.Ittichai Wachiraphiphatkun

[![PlatformIO](https://img.shields.io/badge/PlatformIO-orange?style=flat&logo=platformio)](https://platformio.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B)](https://isocpp.org/) [![Teensy](https://img.shields.io/badge/Teensy-4.1-green?style=flat)](https://www.pjrc.com/teensy/) [![STM32](https://img.shields.io/badge/STM32-F411CE-blue?style=flat&logo=stmicroelectronics)](https://www.st.com/)

## System Architecture

ระบบเป็น **Distributed System** แบบ **Star Topology** เชื่อมต่อผ่านสาย UART (JST-XH 4 pin) แยกอิสระต่อบอร์ด (Point-to-Point) โดย Slave ทุกตัวต่อตรงเข้าหา Master โดยตรง ไม่มีการต่อพ่วงระหว่าง Slave ด้วยกันเอง เพื่อไม่ให้เกิดคอขวดในการประมวลผลของแต่ละบอร์ด

```
Slave 1 (Wheel)  ---UART--->
Slave 2 (Lift)   ---UART--->  Master (Teensy 4.1)
Slave 3 (Sensor) ---UART--->
Slave 4 (Laser)  ---UART--->
```

## 🛠️ Hardware & Tech Stack

- **Master Controller:** Teensy 4.1 (ARM Cortex-M7)
- **Slave Controller:** STM32 Blackpill (F411CE)
- **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)
- **Control Algorithm:** PID Control, Mecanum Kinematics

## 📂 Project Structure

```text
├── 0_master/       # Teensy 4.1: ศูนย์กลางสั่งการ (Hardware Serial แยกคุยกับแต่ละ Slave)
├── 1_slave_wheel/  # STM32: คุมล้อ Mecanum 4 ล้อ (ลูป PID ความถี่สูง)
├── 2_slave_lift/   # STM32: คุมชุดยกโซ่เฟือง 2 ตัว + ชุดแขน 2 ตัว
├── 3_slave_sensor/ # STM32: คุม Relay 4 ตัว + Servo 2 ตัว + อ่าน Limit Switch 7 ตัว
└── 4_slave_laser/  # STM32: อ่าน Laser Check Field 5 ตัว (Active Low) + TOF 3 ตัว (UART)
```

## 🔌 UART Pin Mapping

| Slave           | Slave Pin (TX/RX) | Master Pin (RX/TX)      |
|-----------------|--------------------|--------------------------|
| 1_slave_wheel   | PA9(tx1)/PA10(rx1) | 0(rx1)/1(tx1)   — Serial1 |
| 2_slave_lift    | PA9(tx1)/PA10(rx1) | 28(rx7)/29(tx7) — Serial7 |
| 3_slave_sensor  | PA9(tx1)/PA10(rx1) | 21(rx5)/20(tx5) — Serial5 |
| 4_slave_laser   | PA9(tx1)/PA10(rx1) | 34(rx8)/35(tx8) — Serial8 |

> หมายเหตุ: ตรวจสอบทิศทาง TX->RX และ RX->TX ให้ถูกต้องเสมอ

## 📡 Communication Protocol

ทุกเฟรมขึ้นต้นด้วย `PROTOCOL_START_BYTE (0xAA)` และปิดท้ายด้วย checksum แบบ XOR ไล่ทีละไบต์ของ payload (`calculateChecksum()`) เพื่อกันข้อมูลเพี้ยนจากสัญญาณรบกวนบนสาย UART

**สาย wheel (Master ↔ 1_slave_wheel)** — คำสั่งเดียว ไม่ต้องแยกชนิด:
```
START | LEN | PAYLOAD (WheelCommand: vx, vy, omega) | CHECKSUM
```

**สาย sensor (Master ↔ 3_slave_sensor)** — รวม 2 คำสั่ง (Servo, Relay) บนสายเดียวกัน จึงมี **Command ID** แทรกเพื่อแยกแยะ:
```
START | CMD_ID | LEN | PAYLOAD (ServoCommand หรือ RelayCommand) | CHECKSUM
```
- `CMD_SERVO (0x01)` → payload คือ `{armAngle, spinAngle}` (0-180 องศา)
- `CMD_RELAY (0x02)` → payload คือ `relayState` (1 ไบต์ bitmask, bit0-3 = Relay_1-4, ใช้ค่า Active LOW: สั่ง `LOW` = relay ติด)

**Failsafe:** ฝั่ง `1_slave_wheel` เก็บเวลาล่าสุดที่รับเฟรมสมบูรณ์ (`lastReceivedTime`) ถ้าขาดการติดต่อเกิน `WHEEL_CMD_TIMEOUT_MS` (300ms) จะสั่ง `robotDrive.stop()` ทันที ป้องกันหุ่นวิ่งค้างหากสาย UART หลุด

## ✅ Implementation Status

| Environment       | สถานะ | รายละเอียด |
|--------------------|--------|-------------|
| `0_master`         | 🟡 บางส่วน | เปิด UART ครบ (`Serial1`, `Serial5`), ส่งคำสั่งทดสอบ Wheel/Servo/Relay อยู่ ยังไม่มี logic อ่านอินพุตควบคุมจริง (จอย/รีโมท) และยังไม่คุยกับ `2_slave_lift`/`4_slave_laser` |
| `1_slave_wheel`    | 🟢 ใช้งานได้ | รับคำสั่งผ่าน `Serial1`, ขับ Mecanum ด้วย `moveSmooth()`, มี Failsafe timeout |
| `2_slave_lift`     | ⚪ ยังไม่เริ่ม | ไฟล์ยังเป็นเทมเพลตเปล่า |
| `3_slave_sensor`   | 🟢 ใช้งานได้ | รับคำสั่ง Servo + Relay ผ่าน `Serial1` (multiplexed ด้วย Command ID), ควบคุม Servo 2 ตัว + Relay 4 ตัว (Active LOW) จริง |
| `4_slave_laser`    | ⚪ ยังไม่เริ่ม | ไฟล์ยังเป็นเทมเพลตเปล่า |

## 📏 Coding Rules

- `include/` ใช้ `.h` เท่านั้น, `src/` ใช้ `.cpp` เท่านั้น
- เขียนโค้ดให้อ่านง่าย ไล่ตามง่าย มากกว่าเน้นโค้ดสั้น
- ห้ามใช้ `delay()` ทุกบอร์ด ให้ใช้ `millis()` แบบ non-blocking เท่านั้น เพื่อไม่ให้ลูป PID/Mecanum สะดุด

รายละเอียดเพิ่มเติมดูได้ที่ [`CLAUDE.md`](./CLAUDE.md)
