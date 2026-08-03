# Abu_Auto_Thailand

โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์อัตโนมัติ (Auto Robot - R2)
ในการแข่งขัน ABU Robocon พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์
by.Ittichai Wachiraphiphatkun

[![PlatformIO](https://img.shields.io/badge/PlatformIO-orange?style=flat&logo=platformio)](https://platformio.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B)](https://isocpp.org/) [![Teensy](https://img.shields.io/badge/Teensy-4.1-green?style=flat)](https://www.pjrc.com/teensy/) [![STM32](https://img.shields.io/badge/STM32-F411CE%20%2F%20F103C8-blue?style=flat&logo=stmicroelectronics)](https://www.st.com/)

## System Architecture

ระบบเป็น **Distributed System** แบบ **Star Topology** เชื่อมผ่าน UART แยกอิสระต่อบอร์ด
(Point-to-Point) Slave ทุกตัวต่อตรงเข้าหา Master ไม่มีการต่อพ่วงระหว่าง Slave ด้วยกันเอง

```
slave_hub    (Teensy 4.1)  <--UART 460800--> |
slave_wheel  (Blackpill)   <--UART 115200--> | Master (Teensy 4.1)
slave_arm    (Blackpill)   <--UART 115200--> |
slave_lift   (Blackpill)   <--UART 115200--> |
slave_button (Blue Pill)   ---UART 921600--> |  ⚠ master ยังไม่มีตัวอ่าน
                                             +--- I2C 400kHz --- BNO085 IMU
```

RPLiDAR และ TFMini-S ทั้ง 4 ตัวไม่ได้ต่อกับ Master โดยตรง แต่ต่อเข้า `slave_hub`
ซึ่งประมวลผลสแกนให้เสร็จแล้วส่งผลลัพธ์กล่อง/กำแพงมาให้ Master ในแพ็กเก็ตเดียว

`slave_button` เป็นบอร์ดใหม่ที่ยังทำงานไม่ครบวง — ฝั่งบอร์ดเสร็จแล้ว (กดปุ่ม → OLED → ยิง
text ออก `Serial1`) แต่ฝั่ง Master ยังไม่มีโค้ดอ่าน `Serial3` เลย จึงยังสั่งงานอะไรไม่ได้

## 🛠️ Hardware & Tech Stack

- **Master / Sensor Hub:** Teensy 4.1 (ARM Cortex-M7) @ 600 MHz
- **Wheel / Arm / Lift Controller:** STM32 Blackpill (F411CE)
- **Keypad Panel:** STM32 Blue Pill (F103C8T6) + คีย์แพด 4x4 + OLED SH1106 128x64
- **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)
- **Control Algorithm:** PID Control, Mecanum Kinematics, LiDAR line fitting

## 📂 Project Structure

PlatformIO ใช้ single project แยก 6 Environment ด้วย `build_src_filter` แต่ละ env
คอมไพล์เฉพาะโฟลเดอร์ของตัวเองและมองเห็นเฉพาะ header ของตัวเอง (`-I include/<board>`)
ทุกบอร์ดจึงมี `config.h` เป็นของตัวเองได้โดยไม่ชนกัน

```text
├── src/
│   ├── master/        # Teensy 4.1: mission runner, odometry, position control, ทุก link
│   ├── slave_hub/     # Teensy 4.1: RPLiDAR + TFMini-S x4 + relay + servo + switch
│   ├── slave_arm/     # Blackpill: แขน 2 แกน encoder PID + homing ด้วย limit switch
│   ├── slave_wheel/   # Blackpill: Mecanum 4 ล้อ speed PID
│   ├── slave_lift/    # Blackpill: lift หน้า/หลัง position PID + soft down + homing
│   └── slave_button/  # Blue Pill: คีย์แพด 4x4 + OLED, ยิง label ออก Serial1 ทางเดียว
├── include/
│   ├── master/  slave_hub/  slave_arm/  slave_wheel/  slave_lift/  slave_button/
│                       # header แยกต่อบอร์ด
└── platformio.ini     # 6 environment
```

### ไฟล์ที่ต้องแก้บ่อย (ฝั่ง master)

| ไฟล์ | ใช้แก้อะไร |
|------|------------|
| `include/master/config.h` | **ค่า tuning ทุกตัว** — gain, ความเร็ว, timeout, tolerance, threshold TF, baud |
| `src/master/mission_list.cpp` | **ลำดับภารกิจจริง** เริ่มที่ไฟล์นี้ |
| `include/master/mission_steps.h` | vocabulary ของ `*_STEP()` ที่เอาไปเรียงเป็นภารกิจ |
| `src/master/mission_runner.cpp` | พฤติกรรมจริงของแต่ละ step |
| `src/master/console.cpp` | คำสั่งบน Serial Monitor |

ค่า threshold ของการตรวจจับ LiDAR (ความกว้างกล่อง, cluster gap, filter alpha) อยู่ฝั่ง hub
ที่ `src/slave_hub/config.cpp` ไม่ใช่ฝั่ง master

ส่วน gain / ระยะ stroke / ลิมิต PWM ขาลงของ lift อยู่ที่ `include/slave_lift/config.h`
ฝั่ง master เก็บแค่ค่า pulse เป้าหมายกับ timeout ใน `include/master/config.h`

## 🔌 Serial / Pin Mapping

### Master (Teensy 4.1)

| Port | Baud | ปลายทาง |
|------|------|---------|
| `Serial` | 115200 | USB Serial Monitor / console |
| `Serial1` | 460800 | slave_hub |
| `Serial2` | 115200 | slave_wheel |
| `Serial6` | 115200 | slave_arm (TX pin 24 / RX pin 25) |
| `Serial7` | 115200 | slave_lift |
| `Wire` | 400 kHz | BNO085 IMU (SDA 18 / SCL 19) |

### slave_hub (Teensy 4.1)

| Port | Baud | ปลายทาง |
|------|------|---------|
| `Serial1` | 460800 | Master (RX1 = 0, TX1 = 1) |
| `Serial8` | 460800 | RPLiDAR (RX8 = 34, TX8 = 35) |
| `Serial2` / `Serial6` / `Serial7` | 115200 | TFMini-S 1 / 2 / 3 |
| `Serial3` | 115200 | TFMini-S 4 (RX3 = 15, TX3 = 14) |

I/O: Relay 1-4 = pin 2/3/4/5 (LOW = ON), arm servo = 6, spin servo = 10,
input ทั้งหมด `INPUT_PULLUP` และ LOW = ACTIVE —
L_SW_Front 12, LDR1 13, LDR2 18, R_SW_Front 17, laser5 19,
SW_Green 20, SW_Blue 21, SW_Red 22, SW_Yellow 23

### slave_wheel / slave_arm / slave_lift (Blackpill F411CE)

ทั้งสามบอร์ดคุยกับ Master ที่ `Serial1` = PA9 (TX) / PA10 (RX) และใช้ USB CDC
เป็น `Serial` สำหรับเมนูทดสอบไปพร้อมกัน

| | FL / Bottom | FR | RL | RR / Top |
|---|---|---|---|---|
| wheel motor A/B | PA1 / PA0 | PA2 / PA3 | PA6 / PA7 | PB0 / PB1 |
| wheel encoder A/B | PB6 / PB7 | PB5 / PB4 | PB14 / PB15 | PB13 / PB12 |
| arm motor A/B | PA7 / PA6 | — | — | PB0 / PB1 |
| arm encoder A/B | PB15 / PB14 | — | — | PB12 / PB13 |

arm limit switch: BOTTOM front PB6 / back PB7, TOP front PB4 / back PB5

lift เป็นคนละบอร์ดกับ arm แต่ผังขาใกล้กัน (ขามอเตอร์ A/B ของเสาหน้าสลับข้างกับ arm top):

| | Front | Back |
|---|---|---|
| lift motor A/B | PB1 / PB0 | PA7 / PA6 |
| lift encoder A/B | PB12 / PB13 | PB15 / PB14 |
| lift limit TOP / BOTTOM | PB5 / PB4 | PB7 / PB6 |

encoder ของ lift นับบวก = ขึ้น, PWM ลบ = ขึ้น / PWM บวก = ลง
โดยขาลงจะถูกจำกัดที่ `*_DOWN_PWM_MAX` ค่อย ๆ ไต่ขึ้นทีละ `DOWN_ACCEL_STEP`
และลดกำลังเป็นเส้นตรงในช่วง `DOWN_BRAKE_ZONE_PULSE` สุดท้ายก่อนถึงเป้าหมาย

### slave_button (Blue Pill F103C8)

`Serial` เป็น USB CDC ใช้ดู log ได้พร้อมกับที่ `Serial1` (PA9 TX / PA10 RX) ยิงไปหา Master
เหมือนบอร์ด Blackpill แต่ core ของ F103 ต้องมี `-D ENABLE_HWSERIAL1` ถึงจะมี `Serial1` ให้ใช้

| ส่วน | ขา |
|------|-----|
| OLED SH1106 128x64 (software I2C) | SCL PB10 / SDA PB11 |
| keypad column (OUTPUT, idle HIGH) | C1 PA2 / C2 PA3 / C3 PA0 / C4 PA1 |
| keypad row (`INPUT_PULLUP`) | R1 PB0 / R2 PB1 / R3 PA6 / R4 PA7 |
| ลิงก์ไป Master | `Serial1` TX PA9 → Teensy `Serial3` RX (pin 15) |
| onboard LED | PC13 (active LOW, ยังไม่ได้ใช้ในโค้ด) |

สแกนคีย์แพดโดยปล่อย column เป็น LOW ทีละเส้นแล้วอ่าน row — ปุ่มที่กดจะอ่านได้ LOW
debounce 5 ms และนับเฉพาะขอบขาลง กดค้างไม่นับซ้ำ ปล่อยปุ่มไม่ส่งอะไร

| | C1 | C2 | C3 | C4 |
|---|---|---|---|---|
| **R1** | A1 | A2 | A3 | A4 |
| **R2** | A5 | A6 | A7 | A8 |
| **R3** | B1 | B2 | B3 | B4 |
| **R4** | B5 | B6 | B7 | B8 |

ค่า pin ทั้งหมดอยู่ใน `include/slave_button/config.h`

## 📡 Protocol

| Link | Master ส่ง | Slave ตอบ |
|------|-----------|-----------|
| wheel | `T,<FL>,<FR>,<RL>,<RR>` / `S` / `Q` | `F,<ms>,<c x4>,<rpm x4>` ที่ 50 Hz |
| hub | `R1 ON` `ARM 40` `SPIN 100` `MODE BOX` `MODE WALL` `STREAM ON` | `HUB,<ms>,<MODE>,<found>,<dist>,<offset>,<angle>,<width>,<points>,<inputMask>,<relayMask>,<arm>,<spin>,<tf1..tf4>,<tfValidMask>` ที่ 20 Hz |
| arm | `CMD,<seq>,<HOME\|B,deg\|T,deg\|POS,b,t\|STATUS\|STOP\|CLEAR>` | `ACK` / `STATE` / `DONE` / `ERR` (ตอบพร้อม seq เดิม) |
| lift | `LP,<front>,<back>` / `LZ` / `S` | `LIFT_POS,<front>,<back>` ที่ 50 Hz + `LIFT_BUSY` / `LIFT_REACHED,<f>,<b>` / `LIFT_HOMING` / `LIFT_HOME_REACHED` / `LIFT_ERROR,<reason>` |
| button ⚠ | — (ทางเดียว) | `A1`..`A8` / `B1`..`B8` + `\n` ตอนกดปุ่ม |

⚠ ลิงก์ button ยังไม่มีตัวอ่านฝั่ง Master — บอร์ดส่งออกมาลอย ๆ ไปก่อน
และเป็น plain text ล้วน ไม่มี framing/checksum เหมือนลิงก์อื่น
การกด 1 ครั้งจะส่งซ้ำ **5 บรรทัด ห่างกัน 20 ms** เผื่อฝั่งรับพลาดรอบแรก
ตอนเขียนฝั่ง Master ต้องกรองซ้ำเอง (เช่น ทิ้ง label เดิมที่มาภายใน ~200 ms)

## ▶️ Build & Upload

```bash
pio run                        # build ทุกบอร์ด
pio run -e master              # build บอร์ดเดียว
pio run -e master -t upload    # อัปโหลด
pio device monitor -e master   # เปิด serial monitor
pio run -t clean               # ล้าง build
```

ชื่อ env: `master` `slave_hub` `slave_arm` `slave_wheel` `slave_lift` `slave_button`

Blackpill และ Blue Pill ตั้งค่าไว้อัปโหลดผ่าน **ST-Link** ถ้าจะแฟลชผ่าน USB (BOOT0/DFU)
ให้สลับ `upload_protocol` ใน `platformio.ini` เป็น `dfu`

## 🎮 การสั่งงาน

**ปุ่มบนตัวหุ่น (ผ่าน hub):**

| ปุ่ม | ผล |
|------|-----|
| SW_Blue | เริ่ม Mission 2 |
| SW_Red | เริ่ม Mission 3 |
| SW_Yellow | หยุดทุกอย่าง, zero odometry + gyro, arm/spin/lift กลับ home, กระตุกกริปเปอร์เปิด |
| SW_Green | ยังไม่ได้ใช้ |

**แผงคีย์แพด (slave_button):** กดแล้วโชว์ label บน OLED 2 วินาทีแล้วกลับเป็น `-`
พร้อมยิง text ออก `Serial1` — แต่ยังไม่ผูกกับภารกิจใด เพราะ Master ยังไม่มีตัวอ่าน
ทดสอบเองได้ที่ `pio device monitor -e slave_button` จะเห็น `[KEY] A4 x5` ทุกครั้งที่กด

**คำสั่งที่ใช้บ่อยบน Serial Monitor ของ master** (พิมพ์ `h` เพื่อดูทั้งหมด):

```
M / M1 / M2      เริ่มภารกิจ (M = ภารกิจที่เลือกไว้)
A                ยกเลิกภารกิจ
s                หยุดฉุกเฉิน
V,0.2,0,0.3      สั่งความเร็ว local (vx, vy m/s, wz rad/s)
G,0.2,0,0.3      สั่งความเร็ว global (field-centric)
P,1.0,0.5,90     สั่งไปตำแหน่ง X, Y (m) และ Yaw (deg)
LP,1500,1500     สั่ง lift / LZ = lift home
AH / AB 90       arm home / arm bottom 90 deg
i                สถานะ I/O ของ hub
r / z            reset yaw ของ gyro / zero odometry
```

**คำสั่งบน Serial Monitor ของ slave_lift** (คำสั่งชุดเดียวกันนี้รับจาก Master ทาง `Serial1` ด้วย):

```
LP,3800,3900     สั่งตำแหน่ง front / back เป็น pulse (clamp 0..*_PULSE_MAX)
LZ / HOME        homing ลงไปชน limit ล่างแล้ว zero encoder ทั้งสองเสา
S / Z            หยุด / หยุดพร้อม zero encoder ทันที
POSE / LIMIT     อ่านตำแหน่ง+PWM+ลิมิต / อ่านเฉพาะลิมิต
PID              โชว์ gain ปัจจุบัน
5300p 4000i 0d   ตั้ง gain ทั้งสองเสาพร้อมกัน (พิมพ์เป็นจำนวนเต็ม x1000 = 5.300)
5300fp 2300bp    ตั้งแยกเสา front (f*) / back (b*)
```

บอร์ด lift พิมพ์ `POSE` อัตโนมัติทุก 500 ms และส่ง `LIFT_POS` ให้ Master ทุก 20 ms

## 📄 License

MIT License — ดูรายละเอียดใน [LICENSE](LICENSE)
