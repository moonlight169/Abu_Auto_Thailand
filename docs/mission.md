# Mission — ระบบลำดับภารกิจของ Master

เอกสารนี้อธิบายว่าภารกิจ (mission) ทำงานยังไง มี step อะไรให้ใช้บ้าง แต่ละตัวรับพารามิเตอร์อะไร
และภารกิจที่มีอยู่ตอนนี้ (Mission 1–3) ทำอะไรทีละขั้น

ไฟล์ที่เกี่ยวข้อง:

| ไฟล์ | หน้าที่ |
|------|---------|
| `src/master/mission_list.cpp` | **ลำดับภารกิจจริง** — ไฟล์ที่แก้บ่อยที่สุดเวลาเซ็ตสนาม |
| `include/master/mission_steps.h` | นิยาม `MissionStepType` + มาโคร `*_STEP()` ทั้งหมด |
| `src/master/mission_runner.cpp` | พฤติกรรมจริงของแต่ละ step (start / done / failed) |
| `include/master/config.h` | ค่า timeout, threshold TF, tolerance, ลิมิตความเร็ว |

## หลักการทำงาน

ภารกิจคืออาร์เรย์ของ `MissionStep` เรียงกันเฉย ๆ ตัวรันทำงานแบบ **ทีละ step ห้ามข้าม**:

```
updateMission()  (เรียกทุกลูป)
  ├─ startCurrentMissionStep()    ← เรียกครั้งเดียวตอนเข้า step
  ├─ currentMissionStepFailed()   ← จริงเมื่อไหร่ → stopMission() ทั้งภารกิจ
  └─ currentMissionStepDone()     ← จริงเมื่อไหร่ → missionIndex++ ไป step ถัดไป
```

- step ไหน fail หรือหมด `timeoutMs` = **ภารกิจหยุดทั้งอัน** ไม่มีการข้ามไป step ถัดไป
  ตอนหยุดจะสั่ง `stopRobot()`, ยกเลิก position control / box parking / lift และส่ง `STOP` ให้แขน
- `timeoutMs = 0` หมายถึง **รอตลอดไป** (เช่น `LDR1_WAIT_STEP(0)`)
- ภารกิจจบเมื่อเจอ `END_STEP()` → พิมพ์ `MISSION FINISHED`
- ทุก step พิมพ์ log ขึ้น Serial Monitor เสมอ (`MISSION STEP <n>: ...` / `MISSION STEP COMPLETE: <n>`)
  ดู log นี้เวลาไล่ปัญหาว่าค้างอยู่ step ไหน

โครงสร้างหนึ่งแถว:

```cpp
struct MissionStep {
  MissionStepType type;
  float p1, p2, p3;   // ความหมายขึ้นกับ type
  float maxSpeed;     // ส่วนใหญ่คือความเร็ว แต่บาง step ยืมไปใช้อย่างอื่น
  uint32_t timeoutMs;
};
```

⚠ ไม่ต้องกรอก field เอง ให้ใช้มาโคร `*_STEP()` เท่านั้น เพราะบาง step ยืม field ไปใช้ผิดชื่อ
(เช่น `FORWARD_LZ_BACK_STEP` เอา `maxSpeed` ไปเก็บ "เวลาถอยหลัง (ms)")

## RESET_STEP คืออะไร

`RESET_STEP()` = **step ตั้งต้นระบบพิกัดก่อนเริ่มวิ่ง** เรียก `resetBeforeMission()` ใน
[mission_runner.cpp:121](../src/master/mission_runner.cpp#L121) ซึ่งทำ 4 อย่าง:

1. `stopRobot()` — หยุดล้อทั้งหมด
2. **ล้าง odometry** — `positionX/Y = 0`, เป้าหมาย `targetPositionX/Y/Yaw = 0`,
   ล้าง error สะสมของ PID (`previousErrorX/Y/Yaw`) และปิด `positionControlActive`,
   `boxParkingActive`, `commandActive`
3. ส่ง `LZ` ออก `Serial7` — สั่ง **lift homing** (ลงไปชน limit ล่างแล้ว zero encoder ทั้งสองเสา)
4. `resetGyroYaw()` — ตั้ง yaw ปัจจุบันให้เป็น 0 องศา

พูดง่าย ๆ คือ "ตรงนี้คือจุด (0, 0) และหันหน้า 0 องศา" ทุก `MOVE_STEP` หลังจากนั้นจะนับระยะ
จากจุดนี้ ถ้าลืมใส่ตอนต้นภารกิจ หุ่นจะวิ่งเทียบกับพิกัดเก่าที่ค้างจากรอบที่แล้ว

**ข้อควรรู้ 2 ข้อ**

- step นี้ `done()` คืน `true` ทันที — มัน **ไม่รอ** ให้ lift homing เสร็จ ถ้าต้องการให้เสาลงสุด
  จริง ๆ ก่อนวิ่ง ให้ตาม `LIFT_STEP(...)` หรือ `WAIT_STEP(...)` ต่อท้ายเอง
- ในโค้ดปัจจุบันจึงมักเขียนคู่กันเป็น `RESET_STEP(), LIFT_STEP(100, 200),`

### RESET_STEP_2 ต่างกันตรงไหน

`RESET_STEP_2()` เรียก `resetBeforeMission2()` — **เหมือนกันทุกอย่าง ยกเว้นไม่ส่ง `LZ`**
คือล้างพิกัด + ล้าง PID + zero gyro แต่ **ไม่ยุ่งกับ lift** ปล่อยให้เสาค้างตำแหน่งเดิม

ใช้ตอน "รีเซ็ตพิกัดกลางภารกิจ" หลังเข้าชนกำแพง/ลิมิตสวิตช์เพื่อยึดตำแหน่งอ้างอิงใหม่
ทั้งที่ยังยกของอยู่ — เห็นได้ใน Mission 2/3 ที่ `FRONT_LIMIT_STEP()` ชนแล้วตามด้วย
`RESET_STEP_2()` ถ้าใช้ `RESET_STEP()` ตรงนั้น เสาจะทิ้งตัวลง home ทันทีทั้งที่คีบชิ้นงานอยู่

| | ล้าง odometry + PID | zero gyro | สั่ง `LZ` (lift home) |
|---|---|---|---|
| `RESET_STEP()` | ✅ | ✅ | ✅ |
| `RESET_STEP_2()` | ✅ | ✅ | ❌ |

## รายการ step ทั้งหมด

### การเคลื่อนที่

| มาโคร | พารามิเตอร์ | จบเมื่อ / fail เมื่อ |
|-------|-------------|---------------------|
| `MOVE_STEP(x, y, yawDeg, maxSpeed)` | เป้าหมาย **global frame** (m, m, deg) นับจาก reset ล่าสุด, `maxSpeed` เป็น m/s | เข้า tolerance 0.05 m / 3° ครบ 10 รอบ · fail เมื่อครบ 30 s |
| `FRONT_LIMIT_STEP(speed, timeoutMs)` | เดินหน้าตรงจนลิมิตสวิตช์หน้า `L_SW_FRONT` ถูกกด | สวิตช์กด · fail เมื่อ hub offline หรือหมดเวลา |
| `LASER3_FORWARD_STEP(speed, timeoutMs)` | เดินหน้าจน **TF2** < 15 cm | TF2 ถึงระยะ · fail เมื่อ hub offline / หมดเวลา |
| `LASER4_FORWARD_STEP(speed, timeoutMs)` | เดินหน้าจน **TF1** < 20 cm | TF1 ถึงระยะ · fail เหมือนกัน |
| `LIDAR_PARK_STEP(distanceMm, timeoutMs)` | จอดเข้ากล่องด้วย LiDAR ที่ระยะที่กำหนด | `boxParkingDone()` · fail เมื่อ `boxParkingFailed()` |
| `WALL_ALIGN_STEP(distanceMm, timeoutMs)` | จัดตัวตั้งฉากกำแพงที่ระยะที่กำหนด | เหมือน LiDAR park |
| `LASER5_SLIDE_STEP(slideSpeed, forwardSpeed, timeoutMs)` | สไลด์ข้าง (+เดินหน้านิดหน่อย) จน Laser5 เจอชิ้นงาน | Laser5 ติด · fail เมื่อหมดเวลา |

⚠ `maxSpeed` ของ `MOVE_STEP` ถูก clamp อยู่ในช่วง `0.01 – MAX_POSITION_SPEED (1.0)` m/s
ค่าที่เขียนไว้ตอนนี้เป็น `15` / `20` จึงกลายเป็น **1.0 m/s เต็มสเกล** ทั้งคู่
ถ้าอยากให้ช้าลงจริงต้องใส่เป็นทศนิยม เช่น `0.35`

### lift / แขน / hub

| มาโคร | ทำอะไร | จบเมื่อ |
|-------|--------|---------|
| `LIFT_STEP(front, back)` | ส่ง `LP,<front>,<back>` เป็น pulse | `liftDone()` · timeout 15 s |
| `ARM_HOME_STEP()` | แขน homing | `ARM_TASK_DONE` · timeout 20 s |
| `ARM_BOTTOM_STEP(deg)` / `ARM_TOP_STEP(deg)` | สั่งแกนล่าง / แกนบนของแขน | เหมือนกัน |
| `ARM_POS_STEP(bottomDeg, topDeg)` | สั่งสองแกนพร้อมกัน | เหมือนกัน |
| `HUB_ARM_STEP(deg)` | เซอร์โวแขนบน hub (0–80°) | ทันที (fire-and-forget) |
| `HUB_SPIN_STEP(deg)` | เซอร์โวหมุนบน hub | ทันที |
| `HUB_RELAY_STEP(n, on)` | รีเลย์ 1–4, `1` = ON · **relay 4 = กริปเปอร์, ON = อ้า** | ทันที |

⚠ `HUB_*` เป็นคำสั่งยิงแล้วจบทันที ไม่มี feedback ว่าเซอร์โวถึงมุมหรือยัง
ถ้าต้องรอให้ของจริงขยับตาม ต้องใส่ `WAIT_STEP(...)` ต่อท้ายเอง (ในโค้ดใช้ 500–1000 ms)

### รอ / เงื่อนไข

| มาโคร | ทำอะไร |
|-------|--------|
| `WAIT_STEP(ms)` | หน่วงเวลาเฉย ๆ |
| `HUB_WAIT_INPUT_STEP(bit, active, timeoutMs)` | รออินพุตของ hub ให้เป็นค่าที่ต้องการ |
| `LDR1_WAIT_STEP(timeoutMs)` | ทางลัดของอันบน = รอ `LDR1` เป็น ON · ใส่ `0` = รอไม่มีกำหนด |
| `END_STEP()` | ปิดท้ายอาร์เรย์ — **ต้องมีเสมอ** |

### step รวมท่า (ใช้กับพื้นต่างระดับ)

- **`TF34_EDGE_FRONT_LIFT_STEP(speed, frontPulse, backPulse, timeoutMs)`**
  เดินหน้าโดยคุมล้อซ้าย/ขวาแยกกัน รอ TF3 และ TF4 ยืนยันว่า "เห็นพื้น" (5–15 cm) ก่อน แล้วรอจังหวะ
  ที่แต่ละข้างพ้นขอบ (อ่านไม่เจอพื้นติดกัน 3 ครั้ง) ข้างไหนพ้นก่อนก็หยุดล้อข้างนั้นก่อน
  พอครบสองข้างจึงหยุดหมดแล้วสั่ง `LP,<frontPulse>,<backPulse>` — step จบเมื่อ lift ถึงตำแหน่ง
- **`TF1_FORWARD_BACK_LIFT_STEP(speed, backPulse, timeoutMs)`**
  เดินหน้าจน TF1 เปลี่ยนจาก "เห็นพื้น" → "ไม่เห็นพื้น" (ยืนยัน 3 ครั้ง) แล้วคลานต่ออีก 500 ms
  ที่ 0.12 m/s เพื่อเผื่อระยะ จากนั้นหยุด **คงตำแหน่งเสาหน้าไว้เท่าเดิม** และสั่งเฉพาะเสาหลัง
- **`FORWARD_LZ_BACK_STEP(fwdSpeed, fwdMs, backSpeed, backMs, timeoutMs)`**
  เดินหน้าตามเวลา → หยุด → สั่ง `LZ` และรอ `LIFT_HOME_REACHED` จริง ๆ → ถอยหลังตามเวลา → หยุด
- **`LASER5_PICK_VERIFY_STEP(slideSpeed, forwardSpeed, armReturnAngle, timeoutMs)`**
  สไลด์หาชิ้นงาน พอ Laser5 ติดก็ปิดรีเลย์ 4 (หุบกริปเปอร์) + สั่ง `ARM 0` รอ 500 ms
  แล้ว **เช็ค Laser5 ซ้ำ**: ยังติด = คีบติดจริง จบ step · ไม่ติด = คีบพลาด → อ้ากริปเปอร์,
  รอ 500 ms, ยืดแขนกลับไปที่ `armReturnAngle`, รออีก 500 ms แล้ววนกลับไปสไลด์หาใหม่จนหมดเวลา
  (`armReturnAngle` แยกตามสนามได้ เพราะ Mission 2 ใช้ 71° ส่วน Mission 3 ใช้ 72°)

## ภารกิจที่มีอยู่

ลงทะเบียนไว้ใน `missionPrograms[]` ท้ายไฟล์ `mission_list.cpp`

### Mission 1 — เก็บ/วางกล่องด้วย LiDAR + ลงพื้นต่างระดับ

1. reset → ยกเสาต่ำ → `MOVE_STEP(3.8, 0.5, 0)` วิ่งไปหน้ากล่อง
2. ไต่เสาขึ้นเป็นสเต็ป (1000 → 2000 → 3800) แล้ว `LIDAR_PARK_STEP(385)` จอดเข้ากล่อง
3. ชุดหยิบชิ้นที่ 1: relay1 ON → แขนล่าง 5° / บน 25° → relay2 ON → ยกแขนขึ้น 100° →
   แขนล่าง 180° → relay1 OFF
4. ลดเสาหน้าลง 0 → `LASER3_FORWARD_STEP` → เสาลง 0 ทั้งคู่ → `LASER4_FORWARD_STEP`
5. ชนลิมิตหน้า → ยกเสา 2000/2100 → จอด LiDAR อีกรอบ → ชุดหยิบชิ้นที่ 2 (relay3)
6. reset พิกัดกลางสนาม → จอด LiDAR → ชุดสลับชิ้นที่ 3
7. ทำท่าลงพื้นต่างระดับ **3 รอบ** รอบละ:
   `LIFT_STEP(0, 50)` → `TF34_EDGE_FRONT_LIFT_STEP` → `TF1_FORWARD_BACK_LIFT_STEP` → `FORWARD_LZ_BACK_STEP`

### Mission 2 — ประกอบอาวุธ สนามฟ้า

| # | step | ความหมาย |
|---|------|----------|
| 1 | `RESET_STEP()` | รีเซ็ตพิกัด + gyro + สั่ง lift home |
| 2 | `HUB_ARM_STEP(5)` | แขน hub เก็บที่ 5° |
| 3 | `HUB_SPIN_STEP(0)` | เซอร์โวหมุนกลับ 0° |
| 4 | `HUB_RELAY_STEP(4, 1)` | อ้ากริปเปอร์รอหยิบ |
| 5 | `LIFT_STEP(2300, 2400)` | ยกตัว หน้า 2300 หลัง 2400 |
| 6 | `MOVE_STEP(1.4, -1.0, 0, 15)` | วิ่ง global frame x 1.4 m, y −1.0 m, yaw 0 |
| 7 | `HUB_ARM_STEP(71)` | กางแขน hub 71° เตรียมหยิบ |
| 8 | `FRONT_LIMIT_STEP(0.4, 15 s)` | คลานเข้าจนชนลิมิตหน้า |
| 9 | `RESET_STEP_2()` | ยึดจุดที่ชนเป็นพิกัดใหม่ **โดยไม่ปล่อยเสาลง** |
| 10 | `LASER5_PICK_VERIFY_STEP(-0.25, +0.04, 71, 15 s)` | สไลด์ซ้าย (ค่าลบ) หาชิ้นงาน คีบแล้วเช็คซ้ำ |
| 11 | `LIFT_STEP(2550, 2650)` | ยกชิ้นงานขึ้น |
| 12–15 | `HUB_ARM_STEP(10)` → wait 1 s → `HUB_SPIN_STEP(133)` → wait 1 s | หุบแขนแล้วหมุนไปฝั่งประกอบ |
| 16 | `LDR1_WAIT_STEP(0)` | **รอ LDR1 ตลอดไป** จนกว่าจะตรวจเจอ |
| 17 | `HUB_RELAY_STEP(4, 1)` | อ้ากริปเปอร์ปล่อยชิ้นงาน |
| 18 | `END_STEP()` | จบ |

### Mission 3 — ประกอบอาวุธ สนามแดง

โครงเดียวกับ Mission 2 เป๊ะ ต่างกันแค่ค่าที่พลิกตามสนาม:

| | Mission 2 (ฟ้า) | Mission 3 (แดง) |
|---|---|---|
| `MOVE_STEP` | `1.4, -1.0` | `1.4, 0.5` |
| มุมแขน hub ตอนหยิบ | `71°` | `72°` |
| ทิศสไลด์ของ `LASER5_PICK_VERIFY` | `-0.25` (ซ้าย) | `+0.25` (ขวา) |

## วิธีสั่งรัน

| วิธี | ผล |
|------|-----|
| Serial Monitor: `M1` / `M2` / `M3` | เลือกแล้วเริ่มภารกิจนั้น (`M` เฉย ๆ = ภารกิจที่เลือกค้างไว้) |
| Serial Monitor: `A` | ยกเลิกภารกิจ |
| ปุ่ม SW_Blue บนหุ่น | เริ่ม Mission 2 |
| ปุ่ม SW_Red บนหุ่น | เริ่ม Mission 3 |
| ปุ่ม SW_Yellow บนหุ่น | หยุดทุกอย่าง + zero odometry/gyro + arm/spin/lift กลับ home + กระตุกกริปเปอร์เปิด 500 ms |

เปลี่ยนภารกิจระหว่างที่กำลังรันอยู่ไม่ได้ — ต้อง `A` หรือกด SW_Yellow ก่อน

## เพิ่มของใหม่

**เพิ่มภารกิจ** — เขียนอาร์เรย์ใหม่ใน `mission_list.cpp` แล้วลงทะเบียนท้ายไฟล์:

```cpp
const MissionStep mission4[] = {
  RESET_STEP(),
  // ...
  END_STEP()
};

const MissionProgram missionPrograms[] = {
  { "MISSION 1", mission1, ARRAY_COUNT(mission1) },
  // ...
  { "MISSION 4", mission4, ARRAY_COUNT(mission4) },
};
```

**เพิ่มชนิด step ใหม่** ต้องแก้ 3 ที่:

1. เพิ่มค่าใน `enum MissionStepType` (`mission_steps.h`)
2. เพิ่มมาโคร `*_STEP()` ของมัน (`mission_steps.h`)
3. เพิ่ม `case` ให้ครบทั้ง `startCurrentMissionStep()`, `currentMissionStepDone()`
   และ `currentMissionStepFailed()` ใน `mission_runner.cpp`

ถ้าลืมข้อ 3 step จะ "เริ่มแล้วไม่มีวันจบ" เพราะ `currentMissionStepDone()` ตกไปที่ `return false`
