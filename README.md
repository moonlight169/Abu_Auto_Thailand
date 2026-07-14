# Abu_Auto_Thailand

โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์อัตโนมัติ (Auto Robot - R2)
ในการแข่งขัน ABU Robocon พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์
by.Ittichai Wachiraphiphatkun

[![PlatformIO](https://img.shields.io/badge/PlatformIO-orange?style=flat&logo=platformio)](https://platformio.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B)](https://isocpp.org/) [![Teensy](https://img.shields.io/badge/Teensy-4.1-green?style=flat)](https://www.pjrc.com/teensy/) [![STM32](https://img.shields.io/badge/STM32-F411CE-blue?style=flat&logo=stmicroelectronics)](https://www.st.com/)

## System Architecture

ระบบเป็น **Distributed System** แบบ **Star Topology** เชื่อมต่อผ่านสาย UART (JST-XH 4 pin) แยกอิสระต่อบอร์ด (Point-to-Point) โดย Slave ทุกตัวต่อตรงเข้าหา Master โดยตรง ไม่มีการต่อพ่วงระหว่าง Slave ด้วยกันเอง เพื่อไม่ให้เกิดคอขวดในการประมวลผลของแต่ละบอร์ด

```
Slave 1 (Wheel)  ---UART--->
Slave 2 (Arm)    ---UART--->
Slave 3 (Lift)   ---UART--->  Master (Teensy 4.1)
Slave 4 (Sensor) ---UART--->
Slave 5 (Laser)  ---UART--->
```

## 🛠️ Hardware & Tech Stack

- **Master Controller:** Teensy 4.1 (ARM Cortex-M7)
- **Slave Controller:** STM32 Blackpill (F411CE)
- **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)
- **Control Algorithm:** PID Control, Mecanum Kinematics

## 📂 Project Structure

```text
├── 0_master/       # Teensy 4.1: ศูนย์กลางสั่งการ (HardwareSerial 5 ช่อง แยกคุยกับแต่ละ Slave)
├── 1_slave_wheel/  # STM32: คุมล้อ Mecanum 4 ล้อ (ลูป PID ความถี่สูง)
├── 2_slave_arm/    # STM32: คุมชุดแขน (Up/Down) up คือหมุนคีบ down หมุนตัวคีบ + Limit Switch 4 ตัว (front/back)
├── 3_slave_lift/   # STM32: คุมชุดยก (Front/back) + Limit Switch 4 ตัว (up/down)
├── 4_slave_sensor/ # STM32: คุม Relay 4 ตัว + Servo 2 ตัว + Limit Switch 1 ตัว (fontRobot) + Light 2 ตัว + TOF 1 ตัว
└── 5_slave_laser/  # STM32: อ่าน Laser Check Field 5 ตัว + TOF 2 ตัว (UART)
```

> เหตุผลที่แยก `2_slave_lift` เดิมออกเป็น `2_slave_arm` + `3_slave_lift`: limit switch ของมอเตอร์ต้องอยู่บอร์ดเดียวกับมอเตอร์ที่มันควบคุม เพื่อตัด latency จากการส่งผ่าน UART ไป-กลับผ่าน master ซึ่งทำให้หยุดมอเตอร์ไม่ทัน

## 🔌 UART Pin Mapping

| Slave           | Slave Pin (TX/RX) | Master Pin (RX/TX)      |
|-----------------|--------------------|--------------------------|
| 1_slave_wheel   | PA9(TX)/PA10(RX)   | 0(RX1)/1(TX1)   — Serial1 |
| 2_slave_arm     | PA9(TX)/PA10(RX)   | 7(RX2)/8(TX2)   — Serial2 |
| 3_slave_lift    | PA9(TX)/PA10(RX)   | 28(RX7)/29(TX7) — Serial7 |
| 4_slave_sensor  | PA9(TX)/PA10(RX)   | 16(RX4)/17(TX4) — Serial4 |
| 5_slave_laser   | PA9(TX)/PA10(RX)   | 21(RX5)/20(TX5) — Serial5 |

> หมายเหตุ: ตรวจสอบทิศทาง TX->RX และ RX->TX ให้ถูกต้องเสมอ

## 📡 Communication Protocol

ทุกเฟรมขึ้นต้นด้วย `PROTOCOL_START_BYTE (0xAA)` และปิดท้ายด้วย checksum แบบ XOR ไล่ทีละไบต์ของ payload (`calculateChecksum()`) เพื่อกันข้อมูลเพี้ยนจากสัญญาณรบกวนบนสาย UART

**สาย wheel (Master ↔ 1_slave_wheel)** — คำสั่งเดียว ไม่ต้องแยกชนิด:
```
START | LEN | PAYLOAD (WheelCommand: vx, vy, omega) | CHECKSUM
```

**สาย sensor (Master ↔ 4_slave_sensor)** — รวม 2 คำสั่ง (Servo, Relay) บนสายเดียวกัน จึงมี **Command ID** แทรกเพื่อแยกแยะ:
```
START | CMD_ID | LEN | PAYLOAD (ServoCommand หรือ RelayCommand) | CHECKSUM
```
- `CMD_SERVO (0x01)` → payload คือ `{armAngle, spinAngle}` (0-180 องศา)
- `CMD_RELAY (0x02)` → payload คือ `relayState` (1 ไบต์ bitmask, bit0-3 = Relay_1-4, ใช้ค่า Active LOW: สั่ง `LOW` = relay ติด)

**Failsafe:** ฝั่ง `1_slave_wheel` เก็บเวลาล่าสุดที่รับเฟรมสมบูรณ์ (`lastReceivedTime`) ถ้าขาดการติดต่อเกิน `WHEEL_CMD_TIMEOUT_MS` (300ms) จะสั่ง `robotDrive.stop()` ทันที ป้องกันหุ่นวิ่งค้างหากสาย UART หลุด

## ✅ Implementation Status

| Environment       | สถานะ | รายละเอียด |
|--------------------|--------|-------------|
| `0_master`         | 🟡 บางส่วน | เปิด UART แค่ `Serial1` (wheel) และ `Serial8` (สายทดสอบไป sensor — ยังไม่ตรงกับ pin mapping ที่ออกแบบไว้คือ `Serial4`) ส่งคำสั่งทดสอบ Wheel/Servo/Relay อยู่ ยังไม่มี logic อ่านอินพุตควบคุมจริง (จอย/รีโมท) และยังไม่คุยกับ `2_slave_arm`/`3_slave_lift`/`5_slave_laser` |
| `1_slave_wheel`    | 🟢 ใช้งานได้ | รับคำสั่งผ่าน `Serial1`, ขับ Mecanum ด้วย `moveSmooth()`, มี Failsafe timeout |
| `2_slave_arm`      | ⚪ ยังไม่เริ่ม | อ่าน Limit Switch 4 ตัว (up/down front/back) ได้แล้ว แต่ loop ควบคุมมอเตอร์ยัง comment ไว้ ยังไม่มี logic จริง |
| `3_slave_lift`     | 🟡 บางส่วน | มี `Lift` class ควบคุม Front/Back ครบ: Homing ชนขา limit ล่างแล้ว reset encoder, PID แยก front/back, Safety Cutoff กันมอเตอร์วิ่งเลย limit, sync RPM front/back ตอน target เท่ากัน — `main.cpp` ยังเป็นโหมดทดสอบ (`liftTo()` hardcode) ยังไม่รับคำสั่งจาก Master ค่า `LIFT_STROKE_MM_*`, `LIFT_SYNC_KP`, `LIFT_HOLD_PWM` ยังต้อง tune จริงบนฮาร์ดแวร์ |
| `4_slave_sensor`   | 🟢 ใช้งานได้ | รับคำสั่ง Servo + Relay ผ่าน `Serial1` (multiplexed ด้วย Command ID), ควบคุม Servo 2 ตัว + Relay 4 ตัว (Active LOW) จริง (ชื่อโฟลเดอร์เปลี่ยนจาก `3_slave_sensor` เดิม) |
| `5_slave_laser`    | ⚪ ยังไม่เริ่ม | ไฟล์ยังเป็นเทมเพลตเปล่า |

## 📏 Coding Rules

- `include/` ใช้ `.h` เท่านั้น, `src/` ใช้ `.cpp` เท่านั้น
- เขียนโค้ดให้อ่านง่าย ไล่ตามง่าย มากกว่าเน้นโค้ดสั้น
- ห้ามใช้ `delay()` ทุกบอร์ด ให้ใช้ `millis()` แบบ non-blocking เท่านั้น เพื่อไม่ให้ลูป PID/Mecanum สะดุด

รายละเอียดเพิ่มเติมดูได้ที่ [`CLAUDE.md`](./CLAUDE.md)
