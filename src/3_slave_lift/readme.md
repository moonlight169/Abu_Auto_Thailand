# 3_slave_lift

บอร์ด STM32 Blackpill (F411CE) คุมชุดยก (Lift) 2 มอเตอร์ (front/back) แยกอิสระต่อกัน แต่ละฝั่งมี encoder และ limit switch (top/bottom) ของตัวเอง

## ไฟล์ในบอร์ดนี้
- `main.cpp` — ประกอบ object `Lift` ตัวเดียว, ผูก interrupt encoder, เรียก `lift.setZero()` ครั้งเดียวตอน `setup()` (auto-home), เรียก `lift.update()` ทุก loop, รับคำสั่ง protocol จาก master (`Serial1`)
- `motor.cpp` — implementation ของ `Motor` (คัดลอกมาจาก `1_slave_wheel`, ต้องมีสำเนาซ้ำในทุก env เพราะ `platformio.ini` ตั้ง `build_src_filter` แยกตามโฟลเดอร์ ห้ามลบ)
- `encoder.cpp` — implementation ของ `Encoder` (คัดลอกแบบเดียวกับ `motor.cpp`)
- `lift.cpp` — implementation ของ `Lift` (ตัวลอจิกหลักของบอร์ดนี้)
- `protocol.cpp` — parser เฟรม UART ฝั่ง lift (`liftReceiverFeed`) รับ `CMD_LIFT_PULSE`/`CMD_LIFT_MM`/`CMD_LIFT_ZERO` จาก master

Header ที่เกี่ยวข้องอยู่ใน `include/`: `motor.h`, `encoder.h`, `lift.h`, `protocol.h`, `config/config_lift.h` (พิน + ค่าคงที่)

## Lift ทำงานยังไง

`Lift` เป็น state machine ไม่มี `delay()` เลย ต้องเรียก `lift.update()` ทุกรอบ `loop()` เสมอ ไม่งั้นมอเตอร์จะไม่ขยับ/ไม่หยุด

มี 3 state: `LIFT_IDLE`, `LIFT_HOMING`, `LIFT_MOVING`

**ทุกครั้งที่ `update()` ถูกเรียก** จะเช็ค limit switch ทั้ง 4 ตัวก่อนเป็นอันดับแรก (`applySafetyCutoff()`) ถ้ามอเตอร์ฝั่งไหนกำลังวิ่งเข้าหา switch ที่โดนกดอยู่ จะสั่งหยุดฝั่งนั้นทันที ไม่ว่า state จะเป็นอะไร กันมอเตอร์ดันชน limit switch ค้าง

### setZero() — โฮมชุดยกกลับตำแหน่งศูนย์
1. เข้า state `LIFT_HOMING`
2. `update()` จะสั่งมอเตอร์ front/back วิ่งลง (`LIFT_HOME_SPEED`) อิสระต่อกัน จนกว่า limit switch bottom ของฝั่งนั้นโดนกด
3. ฝั่งไหนถึง bottom ก่อน จะหยุดฝั่งนั้นและ reset encoder ฝั่งนั้นเป็น 0 ทันที (ไม่ต้องรออีกฝั่ง)
4. เมื่อครบทั้ง front และ back แล้ว กลับเข้า `LIFT_IDLE`

เรียกครั้งเดียวตอน `setup()` ในตอนนี้ (auto-home ตอนบอร์ดสตาร์ท)

### liftTo(pulse) — สั่งไปตำแหน่ง pulse ที่ต้องการ และ "เลี้ยง" ค้างไว้
1. เก็บ `pulse` เป็นเป้าหมาย เข้า state `LIFT_MOVING`
2. `update()` คำนวณ PID แยกฝั่ง front/back อิสระต่อกัน (`_pidFront`, `_pidBack` ใน `lift.cpp`) จาก error (`เป้าหมาย - ค่า encoder ปัจจุบัน`) แล้วสั่ง PWM ตรงจากผลลัพธ์ PID ทันที
3. **ไม่มีการหยุดมอเตอร์อัตโนมัติเมื่อถึงเป้าหมาย** — state ยังอยู่ `LIFT_MOVING` ตลอดเพื่อให้ I-term ของ PID คอยต้านโหลด/แรงโน้มถ่วงที่ทำให้ชุดยกไหลลง (ที่หนักแล้วไหล) ต้องเรียก `lift.stop()` เองถ้าต้องการปล่อยมอเตอร์จริงๆ
4. ใช้ `lift.atTarget()` เพื่อเช็คว่าเข้าใกล้เป้าหมายแล้วหรือยัง (ไม่ได้แปลว่ามอเตอร์หยุด — แค่ error อยู่ในช่วง `LIFT_PULSE_TOLERANCE = 5`)

ค่าคุม PID แยกชุดกันระหว่าง front (`LIFT_PID_KP/KI/KD_FRONT`) กับ back (`LIFT_PID_KP/KI/KD_BACK`) ใน `lift.cpp` เพราะสองฝั่งรับน้ำหนักไม่เท่ากัน (ฝั่งที่หนักกว่าจะไหลมากกว่า ต้องการ `Ki` สูงกว่า) **tune จากฮาร์ดแวร์จริงแล้ว** และค่าไม่เท่ากันตามที่ออกแบบไว้ (front `Kp=1.6/Ki=0.045/Kd=0`, back `Kp=0.55/Ki=0.005/Kd=0`) — front รับน้ำหนักมากกว่าจึงตั้ง `Kp`/`Ki` สูงกว่า back

**หมายเหตุ:** encoder ไม่ได้แปลงเป็นระยะจริง (ไม่มี PPR) นับ pulse ดิบเทียบกับ 3 ระดับที่กำหนดไว้ใน `config_lift.h`:
```
LIFT_LEVEL_0 = 0
LIFT_LEVEL_1 = 1500
LIFT_LEVEL_2 = 3000
```
⚠️ ช่วง pulse ที่ clamp จริงใน `lift.cpp` (`LIFT_FRONT_PULSE_MAX = 4150`, `LIFT_BACK_PULSE_MAX = 4200`) **ไม่ตรงกับคอมเมนต์ wiring ใน `config_lift.h`** ที่เขียนว่า front 0-4500 / back 0-4100 — ต้องเช็คว่ายึดค่าไหนก่อนตั้ง `LIFT_LEVEL_1/2` หรือ tune เพิ่ม

## วิธีเรียกใช้

```cpp
lift.setZero();               // โฮมกลับศูนย์ (ทำครั้งเดียวตอนบูต หรือเรียกใหม่เมื่อไหร่ก็ได้ถ้า encoder หลุด)
lift.liftTo(LIFT_LEVEL_2);    // สั่งไปที่ระดับ 3000 pulse แล้ว PID จะเลี้ยงค้างไว้ที่จุดนั้นต่อเนื่อง
lift.stop();                  // ยกเลิกคำสั่งปัจจุบัน หยุดมอเตอร์ทันที (เลิกเลี้ยง PID)
lift.isBusy();                // true ตลอดตั้งแต่สั่ง liftTo/setZero จนกว่าจะ stop() (ไม่ได้แปลว่ายังไม่ถึงเป้าหมาย)
lift.atTarget();              // true เมื่อ error ทั้งสองฝั่งอยู่ในช่วง tolerance แล้ว (มอเตอร์ยังทำงานอยู่เพื่อเลี้ยงตำแหน่ง)
lift.getFrontCount();         // ค่า encoder ฝั่ง front ตอนนี้
lift.getBackCount();          // ค่า encoder ฝั่ง back ตอนนี้
```

ห้ามลืมเรียก `lift.update()` ทุกรอบใน `loop()` ไม่งั้นคำสั่งข้างบนจะไม่มีผลอะไรเลย

## ✅ verify กับฮาร์ดแวร์จริงแล้ว
- **ทิศ motor**: convention คือ `speed < 0` = ขึ้น, `speed > 0` = ลง (ยืนยันจาก pseudocode เดิมที่เคย tune กับของจริงแล้ว) `LIFT_MOVING` ใน `lift.cpp` กลับเครื่องหมาย PID output ก่อนส่งเข้า `motor.run()` ให้ตรง convention นี้แล้ว (`run(-frontOutput)` / `run(-backOutput)`)
- **ทิศ encoder verify แล้ว**: pulse count เพิ่มขึ้นตอนยกขึ้น (zero อยู่ล่างสุดหลัง `setZero()`) ตรงกับที่ `liftTo()` คาดหวังไว้ — ปรับ wiring `enc_lift_frontA/B` ใน `config_lift.h` (สลับ A/B) และดัดตำแหน่ง hall sensor front/back จนทิศตรงกันแล้ว
- **ช่วง pulse clamp จริงใน `lift.cpp`**: front 0-4150, back 0-4200 (ต่างจากคอมเมนต์ wiring ใน `config_lift.h` ที่เขียนไว้ 0-4500/0-4100 — ดูหมายเหตุด้านบน) — `LIFT_LEVEL_1/2` ปรับให้อยู่ในช่วงนี้แล้วพร้อมเผื่อระยะก่อนชน limit switch บน
- ค่า `LIFT_HOME_SPEED`, `LIFT_PID_KP/KI/KD_FRONT`, `LIFT_PID_KP/KI/KD_BACK` และ `LIFT_PULSE_TOLERANCE` ใน `lift.cpp` tune จากฮาร์ดแวร์จริงแล้ว (ดูค่าด้านบน)

## UART Protocol (Serial1 PA9/PA10 ↔ master Serial7 พิน 28/29, 115200) ✅ ใช้งานได้แล้ว

รูปแบบเฟรมเหมือน slave ตัวอื่น (`START | CMD_ID | LEN | PAYLOAD | CHECKSUM`) รับคำสั่งได้ 3 แบบจาก `include/protocol.h`:

| CMD_ID | struct | ความหมาย |
|--------|--------|----------|
| `CMD_LIFT_PULSE` (0x09) | `LiftPulseCommand {pulseFront, pulseBack}` (long) | เรียก `liftAll.liftTo(pulseFront, pulseBack)` ตรงๆ |
| `CMD_LIFT_MM` (0x0A) | `LiftMMCommand {mmFront, mmBack}` (float) | เรียก `liftAll.liftToMM(mmFront, mmBack)` แปลง มม. → pulse ด้วย `LIFT_STROKE_MM_FRONT/BACK` |
| `CMD_LIFT_ZERO` (0x0B) | `LiftZeroCommand {trigger}` (byte เดียว ไว้ให้ผ่าน framing เฉยๆ) | เรียก `liftAll.setZero()` ใหม่ |

ฝั่ง master ส่งด้วย `sendLiftCommandPulse()`/`sendLiftCommandMM()`/`sendLiftCommandZero()` (`src/0_master/protocol.cpp`) — ตอนนี้ `src/0_master/main.cpp` ส่ง `sendLiftCommandMM(Serial7, 0, 0)` แบบ hardcode ทุก 500ms (`LIFT_SEND_HZ = 2`) ยังไม่มี logic สั่งจากอินพุตควบคุมจริง

## ยังไม่ได้ทำ / TODO ที่เหลือ
- `LIFT_STROKE_MM_FRONT`/`LIFT_STROKE_MM_BACK` ใน `lift.cpp` (ตอนนี้ตั้ง placeholder ไว้ 400mm ทั้งคู่) ยังไม่ได้วัดระยะชักจริงจาก home ถึง top ของแต่ละฝั่ง — ต้องวัดก่อนใช้ `liftToMM()`/`CMD_LIFT_MM` จริงจัง
- `LIFT_SYNC_KP = 0.05` (gain synchronize RPM front/back ตอน target เท่ากัน) และ `LIFT_HOLD_PWM = 40` (PWM เลี้ยงตอน target = 0) ยังเป็นค่าเริ่มต้น ทำเครื่องหมาย `TODO` รอ tune จริงบนฮาร์ดแวร์
- โค้ด debug/auto-test ใน `main.cpp` (`liftAll.CountDebug()`, `liftAll.PWMDebug()`, การสั่ง `liftToMM(0,0)` อัตโนมัติตอนไม่ busy) ถูกคอมเมนต์ปิดไว้อยู่ ยังไม่ใช้งาน
