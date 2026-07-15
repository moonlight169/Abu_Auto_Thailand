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

Master ยังต่อกับเซนเซอร์ตรง (ไม่ผ่าน Slave): TOF (UART) หน้า/ซ้าย/ขวา 3 ตัว + IMU (I2C) 1 ตัว — ดูสถานะการ implement จริงในหัวข้อ [Implementation Status](#-implementation-status)

## 🛠️ Hardware & Tech Stack

- **Master Controller:** Teensy 4.1 (ARM Cortex-M7) @ 600MHz
- **Slave Controller:** STM32 Blackpill (F411CE)
- **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)
- **Control Algorithm:** PID Control, Mecanum Kinematics

## 📂 Project Structure

PlatformIO ใช้ single project แยก 6 Environment ด้วย `build_src_filter` ใน `platformio.ini` (ไม่ใช่ 6 โฟลเดอร์โปรเจกต์อิสระ) ไฟล์ `.h` ทั้งหมดรวมอยู่ใน `include/` โฟลเดอร์เดียวไม่แยกต่อ Slave:

```text
├── src/
│   ├── 0_master/           # Teensy 4.1: ศูนย์กลางสั่งการ (HardwareSerial 6 ช่อง แยกคุยกับแต่ละ Slave + TOF หน้า 1 ตัว)
│   ├── 1_slave_wheel/      # STM32: คุมล้อ Mecanum 4 ล้อ (Kinematics + PID แยกล้อ ความถี่สูง)
│   ├── 2_slave_arm/        # STM32: คุมชุดแขน 2 มอเตอร์ (Bottom หมุนตัวคีบ / Top หมุนคีบ) + Limit Switch 4 ตัว (front/back x2)
│   ├── 3_slave_lift/       # STM32: คุมชุดยก (Front/Back) + Limit Switch 4 ตัว (top/bottom x2)
│   ├── 4_slave_sensor/     # STM32: คุม Relay 4 ตัว + Servo 2 ตัว (บอร์ดนี้เป็น Sender ฝั่ง Servo/Relay)
│   └── 5_slave_laser/      # STM32: อ่าน Laser Check Field 5 ตัว + Limit Switch 1 ตัว (frontRobot) + Light 2 ตัว (บอร์ดนี้เป็น Reader)
├── include/                # Header (.h) รวมทุก Environment + include/config/ เก็บไฟล์พิน/ค่าคงที่ต่อบอร์ด
└── lib/                    # Library ใช้ร่วมกันทุก Environment: Kinematics (Mecanum) และ PID
```

> โฟลเดอร์ `backup_slave_lift/`, `backup_slave_sensor/`, `backup_slave_wheels/` ที่ root เป็นโค้ดต้นแบบเก่าก่อนแยก `build_src_filter` — ไม่อยู่ใน `platformio.ini` และไม่ถูก build เก็บไว้อ้างอิงเท่านั้น

## 🔌 UART Pin Mapping (ตามที่ออกแบบไว้)

| Slave           | Slave Pin (TX/RX) | Master Pin (RX/TX)      |
|-----------------|--------------------|--------------------------|
| 1_slave_wheel   | PA9(TX)/PA10(RX)   | 0(RX1)/1(TX1)   — Serial1 |
| 2_slave_arm     | PA9(TX)/PA10(RX)   | 7(RX2)/8(TX2)   — Serial2 |
| 3_slave_lift    | PA9(TX)/PA10(RX)   | 28(RX7)/29(TX7) — Serial7 |
| 4_slave_sensor  | PA9(TX)/PA10(RX)   | 16(RX4)/17(TX4) — Serial4 |
| 5_slave_laser   | PA9(TX)/PA10(RX)   | 34(RX8)/35(TX8) — Serial8 |

> ⚠️ **ตอนนี้โค้ดจริงใน `src/0_master/main.cpp` ไม่ตรงตารางนี้:** ส่งคำสั่ง Servo/Relay ให้ `4_slave_sensor` ผ่าน `Serial8` (ไม่ใช่ `Serial4`) และ `src/0_master/laser_sensors.cpp` ก็อ่านข้อมูล Laser/Lsw/Light จาก `Serial8` เหมือนกัน — เท่ากับ `Serial8` ถูกใช้ซ้อนกันทั้งส่งหาสายสอง (sensor) และรับจากสายห้า (laser) ส่วน `Serial4` เปิดไว้เฉยๆ ไม่มีจุดไหนอ่าน/เขียนจริง ต้องแก้ให้ตรงกับตารางออกแบบก่อนต่อสายจริงกับ `4_slave_sensor`/`5_slave_laser` พร้อมกัน

## 📡 Communication Protocol

ทุกเฟรมขึ้นต้นด้วย `PROTOCOL_START_BYTE (0xAA)` และปิดท้ายด้วย checksum แบบ XOR ไล่ทีละไบต์ของ payload (`calculateChecksum()`) เพื่อกันข้อมูลเพี้ยนจากสัญญาณรบกวนบนสาย UART โครงเฟรมนิยามรวมไว้ที่ `include/protocol.h` (struct payload + parser state machine ต่อสาย)

**สาย wheel (Master ↔ 1_slave_wheel)** — คำสั่งเดียว ไม่ต้องแยกชนิด:
```
START | LEN | PAYLOAD (WheelCommand: vx, vy, omega) | CHECKSUM
```

**สายอื่นทั้งหมด (arm, sensor, laser, lift)** — มีหลายคำสั่งบนสายเดียวกัน จึงมี **Command ID** แทรกหลัง start byte:
```
START | CMD_ID | LEN | PAYLOAD | CHECKSUM
```

| CMD_ID | ชื่อ | ทิศทาง | Payload |
|--------|------|--------|---------|
| `0x01` CMD_SERVO | Servo | master → 4_slave_sensor | `{armAngle, spinAngle}` (0-180°) |
| `0x02` CMD_RELAY | Relay | master → 4_slave_sensor | `relayState` (bitmask 1 byte, bit0-3 = Relay_1-4, Active LOW: สั่ง `LOW` = relay ติด) |
| `0x03` CMD_ARM | Arm (มุมอิสระ) | master → 2_slave_arm | `{bottomAngle, topAngle, flags}` — slave ยังรับได้แต่ master ไม่มีฟังก์ชันส่งแล้ว (ดูหมายเหตุด้านล่าง) |
| `0x04` CMD_ARM_FB | Arm Feedback | 2_slave_arm → master | `{bottomAngle, topAngle, status}` ส่งทุก 100ms — master ยังไม่มีโค้ดอ่าน (`armFeedbackReceiverFeed()` ประกาศไว้ใน `protocol.h` แต่ไม่มี implementation) |
| `0x05` CMD_ARM_CODE | Arm (รหัสท่า) | master → 2_slave_arm | `code` 1 byte — ทางที่ **ใช้งานจริง** ตอนนี้ (ผ่าน interlock กันแขนชนกัน) |
| `0x06` CMD_LASER | Laser | 5_slave_laser → master | `LaserData {header[5], data[5]}` digitalRead 5 เซนเซอร์ |
| `0x07` CMD_LSW | Limit Switch หน้าหุ่น | 5_slave_laser → master | `LswData {header[1], data[1]}` |
| `0x08` CMD_LIGHT | Light sensor | 5_slave_laser → master | `LightData {header[2], data[2]}` analogRead 8-bit (0-255) |
| `0x09` CMD_LIFT_PULSE | Lift (หน่วย pulse) | master → 3_slave_lift | `{pulseFront, pulseBack}` (long) |
| `0x0A` CMD_LIFT_MM | Lift (หน่วย มม.) | master → 3_slave_lift | `{mmFront, mmBack}` (float) — map เป็น pulse ด้วย `LIFT_STROKE_MM_FRONT/BACK` |
| `0x0B` CMD_LIFT_ZERO | Lift (สั่งโฮม) | master → 3_slave_lift | `trigger` (byte เดียว ไว้ให้ผ่าน framing เฉยๆ, เรียก `setZero()`) |

**Failsafe:** มีเฉพาะฝั่ง `1_slave_wheel` — เก็บเวลาล่าสุดที่รับเฟรมสมบูรณ์ (`lastReceivedTime`/`g_prev_command_time`) ถ้าขาดการติดต่อเกิน `WHEEL_CMD_TIMEOUT_MS` (300ms) จะสั่งความเร็วเป็น 0 ทันที ป้องกันหุ่นวิ่งค้างหากสาย UART หลุด สายอื่น (arm/lift/sensor/laser) **ยังไม่มี failsafe timeout**

## ✅ Implementation Status

| Environment       | สถานะ | รายละเอียด |
|--------------------|--------|-------------|
| `0_master`         | 🟡 บางส่วน | เปิด UART ครบทุกช่อง (`Serial1` wheel, `Serial2` arm, `Serial7` lift, `Serial8` sensor, `Serial4` laser-ตามคอมเมนต์แต่ไม่ได้ใช้จริง, `Serial6` TOF หน้า) แต่ pin จริงชนกัน (ดูหัวข้อ UART Pin Mapping) ส่งคำสั่งทดสอบ Wheel/Arm/Servo/Relay/Lift แบบ **hardcode ทั้งหมด** ในลูป ยังไม่มี logic อ่านอินพุตควบคุมจริง (จอย/รีโมท) ยังไม่อ่าน Arm Feedback, ยังไม่ใช้ IMU และ TOF ซ้าย/ขวา (โค้ดอ่าน TOF หน้าคอมเมนต์ปิดไว้อยู่) |
| `1_slave_wheel`    | 🟢 ใช้งานได้ | รับ `WheelCommand` ผ่าน `Serial1`, แปลง vx/vy/omega → RPM ต่อล้อด้วย `Kinematics` (Mecanum, `lib/Kinematics`), ปิดลูป PID แยกต่อล้อ 4 ตัวจาก encoder RPM จริง (`lib/PID`) ที่ 80Hz, มี Failsafe timeout 300ms |
| `2_slave_arm`      | 🟢 ใช้งานได้ | State machine ควบคุม Bottom (หมุนตัวคีบ 0-180°, ต้อง calibrate ก่อน) และ Top (หมุนคีบ -90..+90°, อ้างอิง center ตอนบูต) ครบ: calibrate non-blocking, PID เลี้ยงตำแหน่งค้าง, Safety cutoff ชน limit หยุดเฉพาะทิศเข้าหา, Interlock กันแขนชนกันเอง, สั่งงานจาก master ผ่านรหัสท่า `CMD_ARM_CODE` (ใช้งานจริง) หรือคีย์ debug ทาง USB Serial — path มุมอิสระ (`CMD_ARM`) ยังรับได้แต่ข้าม interlock และ master เลิกใช้แล้ว รายละเอียดเพิ่มเติมดู [`src/2_slave_arm/readme.md`](src/2_slave_arm/readme.md) |
| `3_slave_lift`     | 🟢 ใช้งานได้ | `Lift` class คุม Front/Back อิสระ: Homing ชนขา limit ล่าง reset encoder (auto-home ตอนบูต), PID แยก front/back (tune จากฮาร์ดแวร์จริงแล้ว: front 1.6/0.045/0, back 0.55/0.005/0), sync RPM front/back ตอน target เท่ากัน, hold PWM แทน PID ตอน target = 0, Safety Cutoff กันมอเตอร์ชน limit ค้าง, **รับคำสั่งจาก master ผ่าน UART แล้ว** (`CMD_LIFT_PULSE`/`CMD_LIFT_MM`/`CMD_LIFT_ZERO`) ค่า `LIFT_STROKE_MM_FRONT/BACK`, `LIFT_SYNC_KP`, `LIFT_HOLD_PWM` ยังเป็นค่า placeholder ที่ทำเครื่องหมาย `TODO` รอ tune จริงบนฮาร์ดแวร์ รายละเอียดเพิ่มเติมดู [`src/3_slave_lift/readme.md`](src/3_slave_lift/readme.md) |
| `4_slave_sensor`   | 🟢 ใช้งานได้ | รับคำสั่ง Servo + Relay ผ่าน `Serial1` (multiplexed ด้วย Command ID), ควบคุม Servo 2 ตัว + Relay 4 ตัว (Active LOW) จริง |
| `5_slave_laser`    | 🟢 ใช้งานได้ | อ่าน Laser 5 ตัว + Limit Switch หน้าหุ่น 1 ตัว + Light sensor 2 ตัว (`analogReadResolution(8)`) ทุกลูป แล้วส่งกลับ master ทุก 50ms ผ่าน `Serial1` (`CMD_LASER`/`CMD_LSW`/`CMD_LIGHT`) — พิน Laser 1-5 ยังเป็นค่าที่เลือกไว้ล่วงหน้า **ยังไม่ยืนยันกับวงจรจริง** (ดูคอมเมนต์ใน `include/config/config_laser.h`) |

## 🟡 ข้อควรระวัง / งานที่เหลือภาพรวม

- **Serial8/Serial4 ชนกันที่ `0_master`** — ต้องแก้ให้ `4_slave_sensor` ใช้ `Serial4` และ `5_slave_laser` ใช้ `Serial8` ให้ตรงตารางออกแบบก่อนต่อสายทั้งสองเส้นพร้อมกันจริง (ตอนนี้ต่อพร้อมกันจะชนกันบนสาย `Serial8`)
- **`0_master` ยังสั่งงานทุกบอร์ดแบบ hardcode** (wheel หยุดนิ่ง, arm code 0 ค้าง, servo/relay ค่าคงที่, lift 0mm ค้าง) — ยังไม่มี logic รับอินพุตควบคุมจริง (จอย/รีโมท) มาแทนที่
- **Arm Feedback (`CMD_ARM_FB`) ยังไม่ถูกอ่านฝั่ง master** — ต้อง implement `armFeedbackReceiverFeed()` ใน `src/0_master/protocol.cpp` เพิ่มถ้าจะใช้
- **IMU (I2C) และ TOF ซ้าย/ขวา ยังไม่มีโค้ด** — มีแค่ TOF หน้า (`Serial6`, `TFminiS`) ที่ implement ไว้แต่ถูกคอมเมนต์ปิดอยู่ในลูปจริง
- **ค่าคุม (PID/ระยะจริง) ที่ยัง TODO:** `2_slave_arm` (Kp=1, Ki=Kd=0 ทั้งสองแกน, ยังไม่ยืนยัน pulse/องศาจากฮาร์ดแวร์), `3_slave_lift` (`LIFT_STROKE_MM_FRONT/BACK`, `LIFT_SYNC_KP`, `LIFT_HOLD_PWM`)
- **ไม่มี failsafe timeout** สำหรับสาย arm/lift/sensor/laser (มีแค่ wheel) — ถ้าสาย UART หลุดกลางทาง ฝั่ง slave จะค้างค่าล่าสุดไว้ต่อไป

## 📏 Coding Rules

- `include/` ใช้ `.h` เท่านั้น, `src/` ใช้ `.cpp` เท่านั้น
- เขียนโค้ดให้อ่านง่าย ไล่ตามง่าย มากกว่าเน้นโค้ดสั้น
- ห้ามใช้ `delay()` ทุกบอร์ด ให้ใช้ `millis()` แบบ non-blocking เท่านั้น เพื่อไม่ให้ลูป PID/Mecanum สะดุด

รายละเอียดเพิ่มเติมดูได้ที่ [`CLAUDE.md`](./CLAUDE.md)
