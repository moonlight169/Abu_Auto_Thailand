# 2_slave_arm

บอร์ด STM32 Blackpill (F411CE) คุมชุดแขน 2 มอเตอร์ แยกอิสระต่อกัน:

- **Bottom (`_armDown`)** — หมุนตัวคีบ ช่วง 0-180° ใช้ encoder (PB15/PB14) + limit switch front/back (PB7/PB6)
- **Top (`_armUp`)** — หมุนคีบ ช่วง -90° ถึง +90° โดย 0° คือ center (+90° = ไปทาง back, -90° = ไปทาง front) ใช้ encoder (PB12/PB13) + limit switch front/back (PB5/PB4)

limit switch ทุกตัวเป็น Active LOW (INPUT_PULLUP) พินทั้งหมดกำหนดใน `include/config_arm.h`

**สถานะ build:** ✅ `pio run -e 2_slave_arm` ผ่าน และ `0_master` / `1_slave_wheel` / `4_slave_sensor` ที่แชร์ `protocol.h` เดียวกันยัง build ผ่านทั้งหมด

## ไฟล์ในบอร์ดนี้
- `main.cpp` — loop หลัก: เรียก `arm_update()` ทุกรอบ, รับคำสั่ง protocol จาก master (`Serial1`), รับคำสั่ง debug ทาง USB Serial, ส่ง feedback กลับ master ทุก 100ms และพิมพ์สถานะทุก 500ms
- `arm_control.cpp` — ลอจิกหลักของบอร์ดนี้ (state machine ของ bottom/top, calibrate, PID, safety)
- `motor.cpp` — implementation ของ `Motor` (สำเนาซ้ำเหมือน env อื่น เพราะ `build_src_filter` แยกโฟลเดอร์ ห้ามลบ)
- `encoder.cpp` — implementation ของ `Encoder` (สำเนาแบบเดียวกับ `motor.cpp` นับ quadrature ×4 ผ่าน interrupt ทั้งขา A และ B — EXTI line 12/13/14/15 ไม่ชนกัน)
- `protocol.cpp` — สร้าง/parse เฟรม UART ฝั่ง arm (`sendArmFeedback`, `armReceiverFeed`)

Header ที่เกี่ยวข้องใน `include/`: `arm_control.h`, `motor.h`, `encoder.h`, `config_arm.h`, `protocol.h`

## การทำงาน

เป็น state machine ต้องเรียก `arm_update()` ทุกรอบ `loop()` เสมอ ไม่งั้นมอเตอร์จะไม่ขยับ/ไม่หยุด

- Bottom มี 5 state: `BOTTOM_IDLE`, `BOTTOM_CALIBRATING`, `BOTTOM_CALIB_PAUSE`, `BOTTOM_CALIB_BACKOFF`, `BOTTOM_MOVING`
- Top มี 2 state: `TOP_IDLE`, `TOP_MOVING` (ยังไม่มี state calibrate ของ top)

ทิศทาง bottom (ตรวจแล้วสอดคล้องกันทั้งไฟล์): `speed < 0` = วิ่งเข้าหา front (0°), `speed > 0` = วิ่งเข้าหา back (180°)

### bottom_calibrate() — หาตำแหน่งศูนย์ของ bottom
1. เข้า state `BOTTOM_CALIBRATING` สั่งมอเตอร์วิ่งด้วย `BOTTOM_CALIB_SPEED = -80` (เข้าหาฝั่ง front)
2. เมื่อ limit switch (front **หรือ back**) โดนกด → หยุดแล้วเข้า `BOTTOM_CALIB_PAUSE` (นิ่งรอ 300ms) → เข้า `BOTTOM_CALIB_BACKOFF` ถอยออกด้วย speed 40 อีก 300ms → หยุด → reset encoder เป็น 0 → `_bottomCalibrated = true`
3. ถ้าเกิน `CALIB_TIMEOUT_MS = 10000` (10 วินาที) ยังไม่เจอ switch จะหยุดและยกเลิก (calibrated ยังเป็น false)

ทุกขั้นเป็น non-blocking จับเวลาด้วย `millis()` — ระหว่าง calibrate ลูปยังรับ protocol / ส่ง feedback / คุม top ตามปกติ

### bottom_goTo(deg) — สั่ง bottom ไปมุมที่ต้องการ แล้ว "เลี้ยง" ค้างไว้
1. ต้อง calibrate ก่อน ไม่งั้นคำสั่งถูกปฏิเสธ (`[BOTTOM] Not calibrated!`)
2. มุมถูก constrain 0-180° เข้า state `BOTTOM_MOVING`
3. `arm_update()` คำนวณ PID จาก error (`เป้าหมาย × 890 pulse/° − encoder ปัจจุบัน`) แล้วสั่ง PWM ต่อเนื่อง ไม่หยุดเองเมื่อถึงเป้าหมาย (เลี้ยงตำแหน่งแบบเดียวกับ lift) ต้องเรียก `arm_stop()` เองถ้าจะปล่อยมอเตอร์ ดังนั้น `bottom_isBusy()` จะ true ค้างหลังสั่ง goTo จนกว่าจะ stop
4. ระหว่างวิ่งเช็ค limit ตามทิศเป้าหมาย: target > 90° เช็คฝั่ง back, target ≤ 90° เช็คฝั่ง front โดนเมื่อไหร่หยุดและกลับ `BOTTOM_IDLE`

### top_goTo(deg) — สั่ง top ไปมุม -90° ถึง +90°
1. **ไม่ต้อง calibrate** — ถือว่าตำแหน่งตอนเปิดบอร์ด (encoder = 0) คือ center (`_topCenter = 0` เสมอ ยังไม่มีขั้นตอน calibrate ของ top และ `_topCalibrated` ไม่เคยถูกตั้งเป็น true)
2. PID เลี้ยงที่ `_topCenter + deg × (-890)` — `PULSE_PER_DEG_TOP` เป็นค่าลบเพื่อกลับทิศ encoder และ output ถูกกลับเครื่องหมายอีกทีก่อนส่งเข้า `_armUp.run()`
3. **มุมไม่ถูก constrain** (ต่างจาก bottom) — ฝั่งผู้สั่งต้องส่งค่าในช่วง -90 ถึง 90 เอง
4. กติกาพิเศษ: ถ้าเป้าหมายคือ 0° (center) จะ**ยอมให้วิ่งได้เสมอแม้ limit switch โดนกดอยู่** เพราะ center อยู่ระหว่าง limit ทั้งสองฝั่งแน่นอน

### Safety cutoff (`_applySafety()` ทำงานท้าย `arm_update()` ทุกรอบ ไม่ว่า state ไหน)
หลักการ: หยุดมอเตอร์เฉพาะเมื่อกำลังวิ่ง"เข้าหา" limit switch ที่โดนกดอยู่ (วิ่งหนีออกได้เสมอ)
- bottom: `speed < 0` = เข้า front (0°), `speed > 0` = เข้า back (180°)
- top: `speed < 0` = เข้า front (-90°), `speed > 0` = เข้า back (+90°) — ทิศเดียวกับเช็คใน `TOP_MOVING` แล้ว (แก้บั๊กทิศขัดกันไปแล้ว ดูหัวข้อผลตรวจ)
- top ยกเว้นเมื่อเป้าหมายเป็น center (0°) ให้วิ่งได้เสมอแม้ limit โดนกด เพราะ center อยู่ระหว่าง limit สองฝั่ง

### Interlock กันแขนชนกันเอง (`arm_sendCommand()`)
- ขยับ bottom ได้ต่อเมื่อ top อยู่ใกล้ center (|มุม| < 15°)
- สั่ง top ไป +90° ได้ต่อเมื่อ bottom อยู่ใกล้ 0° / สั่ง top ไป -90° ได้ต่อเมื่อ bottom อยู่ใกล้ 180°

interlock ทำงานใน 2 path: คีย์ debug ทาง USB Serial และรหัสจาก master (`sendArmCommand` / `CMD_ARM_CODE`) เพราะทั้งคู่วิ่งเข้า `arm_sendCommand()` ตัวเดียวกัน — ⚠️ เหลือเฉพาะ path ส่งมุมอิสระ (`CMD_ARM`) ที่เรียก `bottom_goTo()`/`top_goTo()` ตรงๆ ข้าม interlock

## UART Protocol (Serial1 PA9/PA10 ↔ master Serial2 พิน 7/8, 115200)

รูปแบบเฟรมเหมือน slave ตัวอื่น (parser ตรวจ cmdId + len + checksum ครบก่อนรับคำสั่ง):

```
[0xAA][cmdId][len][payload ...][checksum]   (checksum = XOR ทุก byte ของ payload)
```

### ArmCommand — master → slave (cmdId = `CMD_ARM` = 0x03, len = 3)

| field | type | ความหมาย (ตามที่ slave implement จริง) |
|-------|------|----------|
| `bottomAngle` | uint8 | มุมเป้าหมาย bottom 0-180° / `0xFF` = ไม่สั่ง bottom ในเฟรมนี้ |
| `topAngle` | uint8 ในเฟรม แต่ slave ตีความเป็น **int8** | มุมเป้าหมาย top -90 ถึง +90° (two's complement เช่น -90 = 0xA6) / `0xFF` (= -1) = ไม่สั่ง top → สั่ง -1° ตรงๆ ไม่ได้ |
| `flags` | uint8 | bit0 = calibrate bottom, bit2 = stop ทั้งหมด (+reset calibrate ถ้ากำลัง calibrate ค้าง), bit3 = reset calibrate — **bit1 (calibrate_top) และ bit4 (top_calib_sequence) มีในคอมเมนต์ `protocol.h` แต่ยังไม่ถูก implement ใน `main.cpp`** |

ในเฟรมเดียวกัน flags ถูกประมวลผล**ก่อน**มุม — ถ้าส่ง stop (bit2) พร้อมมุมมาด้วย มุมจะถูกสั่งทับต่อทันที (stop แล้วขยับต่อ) ควรส่งอย่างใดอย่างหนึ่งต่อเฟรม

### ArmCodeCommand — master → slave (cmdId = `CMD_ARM_CODE` = 0x05, len = 1) ✅ ใช้งานได้แล้ว

master ส่ง**รหัสท่าเป็นตัวเลข** 1 byte ผ่าน `sendArmCommand(port, code)` — ฝั่ง slave เอาเข้า `arm_sendCommand()` ตัวเดียวกับเมนู debug จึงผ่าน **interlock กันแขนชนกันด้วย** (ต่างจาก path `CMD_ARM` แบบส่งมุมที่ยังข้าม interlock)

```cpp
sendArmCommand(Serial2, 0);   // bottom → 0°     (ต้อง top อยู่ center + calibrate แล้ว)
sendArmCommand(Serial2, 1);   // bottom → 180°   (ต้อง top อยู่ center + calibrate แล้ว)
sendArmCommand(Serial2, 2);   // top → 0° (center)
sendArmCommand(Serial2, 3);   // top → +90°      (ต้อง bottom ≈ 0°)
sendArmCommand(Serial2, 4);   // top → -90°      (ต้อง bottom ≈ 180°)
sendArmCommand(Serial2, 9);   // calibrate bottom
```

การทำงานฝั่ง slave เมื่อ master **ส่งรหัสเดิมซ้ำต่อเนื่อง** (master ส่งทุก 50ms แบบเดียวกับ wheel):
- ทำงานจริงครั้งเดียวต่อรหัส (จำ `activeCode` ที่สั่งสำเร็จแล้ว ไม่สั่งซ้ำ ไม่ log spam)
- ถ้าโดน interlock/ยังไม่ calibrate → **retry เองทุก 500ms จนสำเร็จ** เช่น master ส่ง 3 ค้างไว้ระหว่าง bottom กำลังวิ่งไป 0° พอ bottom เข้าเขต ±15° คำสั่ง 3 จะติดเองอัตโนมัติ (`arm_sendCommand()` คืน `bool` บอกว่ารับคำสั่งสำเร็จไหม)
- คำสั่งจากคีย์ debug ทาง USB ยังใช้ได้ปกติ และไม่ถูกรหัสเดิมที่ master ส่งซ้ำมาทับ

### ArmFeedback — slave → master ทุก 100ms (cmdId = `CMD_ARM_FB` = 0x04, len = 3)

| field | type | ความหมาย |
|-------|------|----------|
| `bottomAngle` | uint8 | มุม bottom ปัจจุบัน (constrain 0-255) |
| `topAngle` | int8 | มุม top ปัจจุบัน (two's complement) |
| `status` | uint8 | bit flags ตามตารางล่าง |

| bit | ความหมาย |
|-----|----------|
| 0x01 | bottom calibrated |
| 0x02 | bottom busy (ไม่อยู่ IDLE — true ค้างตลอดที่กำลังเลี้ยงตำแหน่ง) |
| 0x04 | bottom atTarget (error < 10 pulse) |
| 0x08 | top calibrated (**เป็น 0 เสมอ** — ยังไม่มี calibrate ของ top) |
| 0x10 | top busy |
| 0x20 | top atTarget |

ฝั่ง master: `sendArmCommand(port, code)` implement แล้วใน `src/0_master/protocol.cpp` ✅ และ `0_master/main.cpp` ส่งรหัสซ้ำทุก 50ms (แก้รหัสตรง `sendArmCommand(Serial2, ...)` ใน loop) — ส่วน path ส่งมุมอิสระ (`CMD_ARM`) ฝั่ง slave ยังรับได้อยู่แต่ master ไม่มีฟังก์ชันส่งแล้ว และ `armFeedbackReceiverFeed()` (อ่าน feedback) ยังประกาศไว้เฉยๆ ไม่มี implementation ถ้าจะใช้ต้องเขียนเพิ่มใน `src/0_master/protocol.cpp`

## Debug ผ่าน USB Serial (115200)

| key | คำสั่ง |
|-----|--------|
| `0` | bottom → 0° (ผ่าน interlock) |
| `1` | bottom → 180° (ผ่าน interlock) |
| `2` | top → 0° (center) |
| `3` | top → +90° (ผ่าน interlock) |
| `4` | top → -90° (ผ่าน interlock) |
| `9` | calibrate bottom |
| `s` | stop ทั้งหมด (+ reset calibrate ถ้ากำลัง calibrate ค้าง) |

สถานะ (มุม/calibrated/busy ของทั้งสองแกน) พิมพ์อัตโนมัติทุก 500ms อยู่แล้ว (`?` ยังไม่ได้ implement — เป็น case ว่าง)

## ผลตรวจความถูกต้อง

### ✅ ส่วนที่ตรวจแล้วถูกต้อง
- Build ผ่านทุก env ที่เกี่ยวข้อง (arm/master/wheel/sensor) หลังเพิ่ม struct arm ใน `protocol.h`
- โครงเฟรม protocol ตรงกับ convention ของ wheel/sensor (start byte, XOR checksum, ตรวจ len ก่อนรับ payload กัน buffer overflow)
- ทิศทางแกน bottom สอดคล้องกันหมดทั้ง calibrate / MOVING limit check / `_applySafety` (speed<0 = front)
- **ทิศ safety ของ top ใน `_applySafety()` แก้แล้ว** — เดิมทิศขัดกับเช็คใน `TOP_MOVING` (limit ตัวเดียวหยุดทั้งสองทิศ แขนหนีออกจาก limit ไม่ได้ + motor chatter) ตอนนี้ใช้ convention เดียวกันทั้งสองที่: speed > 0 = เข้า back, speed < 0 = เข้า front และเขียนใหม่ด้วยตัวแปรชื่อชัด (`topFrontHit`, `topTowardFront`, ...) อ่านง่ายขึ้น — **ทิศจริงยังต้อง verify กับฮาร์ดแวร์** (ถ้ามอเตอร์/encoder ต่อกลับทิศจากที่โค้ดคาด PID จะวิ่งหนีเป้า ต้องสลับที่ wiring หรือเครื่องหมาย `PULSE_PER_DEG_TOP`)
- **`delay()` ตอน calibrate backoff แก้แล้ว** — เปลี่ยนเป็น sub-state `BOTTOM_CALIB_PAUSE`/`BOTTOM_CALIB_BACKOFF` + `millis()` ไม่มี `delay()` เหลือในบอร์ดนี้แล้ว (ตรงกฎเหล็กข้อ 4)
- Encoder อ่าน/รีเซ็ต count ครอบด้วย `noInterrupts()` ถูกต้อง, EXTI line ไม่ชนกัน
- ไม่มีการหยุดเลี้ยงตำแหน่งเองเมื่อถึงเป้า (ตั้งใจ — เหมือน lift ที่ต้องต้านโหลด)

### 🔴 บั๊กที่ควรแก้ก่อนใช้กับฮาร์ดแวร์จริง

1. **คอมเมนต์ `topAngle` ใน `protocol.h` ไม่ตรงกับ implementation** — คอมเมนต์บอก "0-180°" แต่ slave ตีความเป็น signed int8 (-90..90) ถ้าคนเขียนฝั่ง master เชื่อคอมเมนต์แล้วส่ง 135 slave จะอ่านเป็น -121° แล้ววิ่งผิดทาง ต้องแก้คอมเมนต์ให้ตรง และ `top_goTo()` ไม่ constrain ช่วงมุมด้วย — ค่าเกินช่วงจะไล่ชน limit

### 🟡 ข้อควรระวัง / งานที่เหลือ

- **Interlock ถูกข้ามใน path ส่งมุมอิสระ (`CMD_ARM`)** — slave ยังรับเฟรมแบบส่งมุมตรงอยู่ (แม้ master จะไม่มีฟังก์ชันส่งแล้ว) ถ้าวันหลังเขียนตัวส่งมุมกลับมาใช้ จะสั่งแขนชนกันเองได้ ควรย้ายเงื่อนไข interlock ลงไปใน `bottom_goTo()`/`top_goTo()` ให้กันทุก path หรือใช้แต่รหัสท่า (`CMD_ARM_CODE`) เป็นหลัก
- **tolerance ของ `atTarget` แน่นเกินจริง**: 10 pulse ÷ 890 pulse/° ≈ 0.011° — ด้วย P-only (Kp=1) มอเตอร์อาจ stall ก่อนเข้าเขตนี้ ทำให้ bit atTarget ใน feedback ไม่ติดเลย (เทียบ lift ใช้ 20 pulse จากช่วง ~4500) ควรขยายเป็นหน่วยองศา เช่น 1-2° (≈ 890-1780 pulse)
- **calibrate เช็ค limit ทั้ง front และ back** — ปกติวิ่งไป front แต่ถ้าเปิดบอร์ดตอนแขนกด back switch ค้างอยู่ จะ reset ศูนย์ผิดข้าง (0° ไปอยู่ฝั่ง 180°)
- **top ไม่มี calibrate** — ถ้า encoder หลุด/บอร์ดรีเซ็ตตอนแขนไม่อยู่ center ตำแหน่งจะเพี้ยนโดยไม่มีทางรู้ (`protocol.h` เผื่อ flags bit1/bit4 ไว้แล้วแต่ยังไม่ implement) แนวทาง: หา limit สองฝั่งแล้วเอากึ่งกลางเป็น center
- **ค่าคุมยังเป็น placeholder ยังไม่ tune กับของจริง**: Kp=1, Ki=Kd=0 ทั้งสองแกน และ `PULSE_PER_DEG_BOTTOM/TOP = ±890` ต้องยืนยันจากการวัดจริง (180° = 160,200 pulse)
- `bottom_atTarget()` คำนวณจาก target ล่าสุดแม้อยู่ IDLE — ค่า atTarget ใน feedback ตอนยังไม่เคยสั่ง goTo ไม่มีความหมาย
