# Problem — ปัญหาที่ตรวจเจอแล้วยังไม่ได้แก้

บันทึกจากการไล่อ่านโค้ดฝั่ง master ทั้งหมด (19 ส.ค. 2026, ~5,000 บรรทัด ไม่นับ
`mission_list.cpp`) ทุกข้อในนี้ยืนยันจากโค้ดแล้ว ไม่ใช่การเดา
ข้อที่แก้แล้วให้ย้ายออกจากไฟล์นี้

| # | เรื่อง | ระดับ | สถานะ |
|---|--------|-------|-------|
| 1 | RESET_STEP สั่ง home lift แล้วโดน LIFT_STEP ถัดไปยกเลิกทิ้ง | สูง | ยังไม่แก้ |
| 2 | RESET_STEP ไม่ล้าง `encoderHeadingRad` | กลาง | ยังไม่แก้ |
| 3 | LIDAR park ข้ามการยืนยัน 500 ms ได้ | กลาง | ยังไม่แก้ |
| 4 | `HUB_WAIT_INPUT_STEP(HUB_LDR2, 1, 0)` รอไม่จำกัดเวลา 24 จุด | กลาง | ตั้งใจ? ต้องตัดสินใจ |
| 5 | คำสั่งแขนหายเงียบตอน busy | ต่ำ | ยังไม่แก้ |
| 6 | ขั้วรีเลย์กลับด้านเฉพาะ 2/3 แต่ `hub_buttons` ใช้เลขดิบ | ต่ำ | ยังไม่แก้ |
| 7 | RX overflow เก็บหางบรรทัดไว้ 3 ลิงก์ | ต่ำ | ยังไม่แก้ |
| 8 | `parseHubPacket` ตัด token ที่ 18 เงียบ ๆ | ต่ำ | ยังไม่แก้ |
| 9 | ฟังก์ชัน lift แบบ blocking 3 ตัวไม่มีคนเรียก | ต่ำ | ยังไม่แก้ |
| 10 | คีย์แพดไม่มีปุ่มยกเลิกที่หน้า RECHECK | กลาง | ยังไม่แก้ |

---

## 1. RESET_STEP สั่ง home lift แล้วโดน LIFT_STEP ถัดไปยกเลิกทิ้ง

**ที่มา** `mission_runner.cpp:169` — `resetBeforeMission()` ยิง `Serial7.println("LZ")`
ดิบ ๆ ไม่ผ่าน `startLiftHome()` แล้ว `STEP_RESET` คืน done ทันที (`mission_runner.cpp:534`)
มิชชันจึงเดินต่อโดยไม่รอ homing

**ผลที่แน่นอน** — มี **98 โปรแกรมที่ `LIFT_STEP` ต่อจาก `RESET_STEP()` ทันที**
(ทั้งหมดของ Mode 0) ฝั่ง slave `setTarget()` มีบรรทัด `homing = false;`
(`slave_lift/lift_control.cpp:78`) แปลว่าคำสั่ง `LP` ที่มาถึงในอีก 1-2 ms
ยกเลิก homing ทิ้งก่อนมันจะทำงานเสร็จ **lift จึงไม่เคย home จริงตอนเริ่มมิชชัน**
และ `resetFrontEncoder()` / `resetBackEncoder()` ที่รันเฉพาะตอนแตะลิมิตล่างก็ไม่เคยถูกเรียก
ถ้า encoder ของ lift เพี้ยนสะสม RESET_STEP แก้ให้ไม่ได้ ทั้งที่เจตนาของโค้ดคือจะแก้

**ผลที่ขึ้นกับจังหวะ** — ถ้า lift นอนอยู่บนลิมิตล่างทั้งสองเสาอยู่แล้ว (เช่นเพิ่งกดปุ่มเหลือง)
และมี control tick ของ slave (ทุก 10 ms) แทรกระหว่าง `LZ` กับ `LP` พอดี slave จะส่ง
`LIFT_HOME_REACHED` ออกมาในทิกเดียว ฝั่ง master จับมันที่ `lift_link.cpp:136-141`
ซึ่งเซ็ต `liftTaskStatus = TASK_DONE` **โดยไม่เช็คว่ามีใครสั่ง home อยู่จริงหรือเปล่า**
→ `LIFT_STEP` ที่เพิ่งเริ่มถูกปิดงานทันทีทั้งที่ lift ยังไม่ถึงเป้า

อาการสมมาตรก็มี: `LIFT_REACHED` ที่มาช้า (`lift_link.cpp:106-117`) ปิดงาน home ผิดตัวได้
`STEP_FORWARD_LZ_BACK` กันเคสนี้ไว้แล้วด้วย `if (!liftDone() || !liftHomeReached)`
พร้อมคอมเมนต์อธิบาย — แปลว่ารู้จัก race ตัวนี้แล้ว แต่กันไว้แค่จุดเดียว

**แนวทางแก้** ใน `parseLiftResponse()` แยกสองงานออกจากกันด้วยแฟล็กที่มีอยู่แล้ว
(`startLift()` เซ็ต `liftMoveActive = true`, `startLiftHome()` เซ็ต false):
`LIFT_HOME_REACHED` ปิดงานได้เฉพาะตอน `!liftMoveActive` และ `LIFT_REACHED`
ปิดได้เฉพาะตอน `liftMoveActive` · แล้วให้ `resetBeforeMission()` เรียก `startLiftHome()`
แทนการยิง `LZ` ดิบ ถ้าอยากให้ home จริง

## 2. RESET_STEP ไม่ล้าง encoderHeadingRad

`resetBeforeMission()` และ `resetBeforeMission2()` (`mission_runner.cpp:146-200`)
ล้าง `positionX/Y` และเรียก `resetGyroYaw()` แต่ไม่เคยเรียก `resetOdometry()`
ซึ่งเป็นที่เดียวที่เซ็ต `encoderHeadingRad = 0`

หลัง RESET จึงได้: X/Y = 0, gyro yaw = 0, แต่ **encoder heading ยังค้างค่าเดิม**
ถ้า gyro หลุด (`gyroOnline == false`) `getRobotYawRad()` จะคืนค่าที่ค้างนั้น
ในขณะที่ทุก `MOVE_STEP` ตั้งเป้าโดยสมมติว่า yaw = 0 → เฟรมโลกเอียงไปเท่าค่าที่ค้าง
และตัวคุม yaw จะหมุนหุ่นไปหักล้างมัน

ปุ่ม SW_YELLOW เรียก `resetOdometry()` ถูกต้องอยู่แล้ว (`hub_buttons.cpp:55`)
เป็นหลักฐานว่าฝั่ง RESET_STEP ลืม

## 3. LIDAR park ข้ามการยืนยัน 500 ms ได้

`box_parking.cpp:178-182`

    if (newBoxFrame && boxParkingInToleranceSinceMs == 0)
      boxParkingInToleranceSinceMs = now;

    if ((uint32_t)(now - boxParkingInToleranceSinceMs) >= PARK_DONE_HOLD_MS)

`boxParkingInToleranceSinceMs` ใช้ค่า 0 เป็น sentinel แปลว่า "ยังไม่เข้าเขต"
แต่บรรทัดที่เช็คไม่มี guard `!= 0` → ถ้ารอบแรกที่เข้า tolerance ตรงกับเฟรมเก่า
(`newBoxFrame` เป็น false) ค่าจะยังเป็น 0 แล้ว `now - 0` ก็คือ `millis()`
ซึ่งมากกว่า 500 เสมอ → **จบทันที ไม่ยืนยันเลย**

control loop เดินทุก 50 ms ถ้า box frame จาก HUB มาช้ากว่านั้น เคสนี้เกิดเป็นประจำ
และมี `LIDAR_PARK_STEP` อยู่ 195 จุดในภารกิจที่พึ่งตรงนี้

**แก้** `if (boxParkingInToleranceSinceMs != 0 && (now - boxParkingInToleranceSinceMs) >= PARK_DONE_HOLD_MS)`

## 4. LDR2 รอไม่จำกัดเวลา 24 จุด

`HUB_WAIT_INPUT_STEP(HUB_LDR2, 1, 0)` — พารามิเตอร์ที่ 3 คือ timeout และ `0`
แปลว่ารอตลอดไป ทางออกทางเดียวคือ hub หลุด offline (`mission_runner.cpp:1010`)
ตอนนี้มี 24 จุดในโปรแกรม Mode 1 (12 จุดต่อสนาม) ถ้า LDR2 สกปรกหรือถูกบัง
หุ่นจะค้างที่ step สุดท้ายจนหมดเวลาแข่ง

ถ้าเจตนาคือ "รอจนกว่าคนจะพร้อม" ก็ปล่อยไว้ได้ แต่ควรตัดสินใจให้ชัด
ทางเลือกกลางคือใส่ timeout ยาว ๆ แล้วให้ step fail แทนการค้าง

## 5. คำสั่งแขนหายเงียบตอน busy

`arm_link.cpp:171` — `startArmCommand()` เห็น `armTaskStatus == ARM_TASK_RUNNING`
แล้วพิมพ์ `ARM ERROR: MASTER BUSY` แล้ว `return` **โดยไม่ตั้ง `ARM_TASK_ERROR`**
`armTaskStatus` จึงค้าง RUNNING ของคำสั่งเก่า แล้ว step ใหม่จะไปปิดงานด้วย DONE
ของคำสั่งเก่า = ท่านั้นถูกข้ามไปเงียบ ๆ

เคสที่เกิดจริงและกดได้ง่าย: **กดปุ่มเหลืองแล้วเริ่มมิชชันทันที**
ปุ่มเหลืองเรียก `startArmClearAndHome()` ซึ่งวิ่ง CLEAR ต่อด้วย HOME
ถ้ามิชชันเริ่มระหว่างนั้น `ARM_BOTTOM_STEP` ท่าแรกจะโดนกลืน แล้วแขนไปจบที่ HOME
แทนที่จะเป็น `BOTTOM_FRONT_DEG` — ซึ่งเป็นท่าหมุนแคร่ ผิดตั้งแต่ต้นรัน

**วิธีเลี่ยงตอนนี้** รอ console ขึ้น `ARM RESET: HOME COMPLETE` ก่อนค่อยกดเริ่ม

## 6. ขั้วรีเลย์กลับด้านเฉพาะ 2/3 แต่ hub_buttons ใช้เลขดิบ

ใน `mission_list.h` — `RELAY2_ON = 0` / `RELAY3_ON = 0` (active-low)
แต่ `RELAY1_ON = 1` และ R4 ON = กริปเปอร์อ้า
ส่วน `hub_buttons.cpp:58-60` เขียน `setHubRelay(2, 0)` ซึ่งอ่านเหมือน "ปิดรีเลย์ 2"
แต่ความหมายจริงคือ `RELAY2_ON` = ปล่อยกล่อง ควรใช้ค่าคงที่ที่ตั้งชื่อไว้แล้ว

## 7. RX overflow เก็บหางบรรทัดไว้ 3 ลิงก์

`button_link` ทิ้งทั้งบรรทัดอย่างตั้งใจพร้อมคอมเมนต์อธิบายว่าทำไม (หางบรรทัดอาจเป็น
รหัสที่ถูกต้องได้เอง แล้วหุ่นจะวิ่งตามขยะ) แต่ `hub_link` / `wheel_link` / `lift_link` /
`console` แค่รีเซ็ต index ทำให้หางบรรทัดยาวกลายเป็นเฟรมใหม่
`hub_link` รอดเพราะเช็ค prefix `"HUB,"` อีกชั้น ตัวอื่นความเสี่ยงต่ำแต่ไม่ได้กันไว้

## 8. parseHubPacket ตัด token ที่ 18 เงียบ ๆ

`char *token[18]` คู่กับเงื่อนไข `tokenCount < 18` ถ้าแพ็กเก็ตมีมากกว่า 18 ฟิลด์
ส่วนเกินหายเงียบและยังผ่านเป็นฟอร์แมต 18 token ได้

## 9. โค้ดตาย

`waitLiftAtTarget()` / `moveLiftPulseAndWait()` / `homeLiftAndWait()` ใน `lift_link.cpp`
ไม่มีคนเรียกเลยสักจุด ทั้งสามเป็น blocking loop ที่ `delay(1)` และไม่ service
Serial3/Serial6 ด้วย ถ้าไม่ได้จะใช้ควรลบทิ้ง กันคนหยิบไปใช้ในอนาคต

## 10. คีย์แพดไม่มีปุ่มยกเลิกที่หน้า RECHECK

ปุ่มทั้ง 16 ถูกจองหมดแล้ว และที่หน้า RECHECK มีแค่ปุ่ม 16 (START) ที่ทำงาน —
ปุ่ม 4 (ENTER) ตกที่ `_pageReadyToAdvance()` คืน false ส่วนปุ่มอื่นตกที่ `default: break;`
และหน้านี้ไม่มี timeout ต่างจากหน้า SENT

ถ้ากดผิดจนมาถึงหน้าทวน ทางออกมีแค่กด START ส่งมิชชันผิดออกไป หรือถอดไฟบอร์ด
ซึ่งอันตรายเพราะ master รันทุกรหัสที่ได้รับโดยไม่ถามอะไรเลย

**แนวทางแก้** ให้ปุ่ม 3 (ว่างอยู่บนหน้านี้) เป็น CANCEL เรียก `_resetSelection()`

---

## หมายเหตุ: ปุ่มเหลืองช่วยอะไรได้บ้าง

| ข้อ | กดปุ่มเหลืองก่อนเริ่มช่วยไหม |
|---|---|
| 1 | ไม่ช่วย และ**เพิ่มโอกาสยิง** — มันทำให้ lift ไปนอนบนลิมิตล่างซึ่งเป็นเงื่อนไขของ race พอดี |
| 2 | ช่วย — `resetOdometry()` ล้าง `encoderHeadingRad` ให้จริง |
| 3 | ไม่เกี่ยวเลย |
| 4 | กันไม่ได้ แต่ใช้กดหนีตอนค้างได้ (`stopMission("SW_Y RESET")`) |
| 5 | **สร้างปัญหา** ถ้าเริ่มมิชชันเร็วเกินไป (ดูข้อ 5) |

ข้อดีจริงข้อเดียวที่ไม่มีทางอื่นทดแทน: `onSwitchYellow()` เรียก `startLiftHome()`
โดยไม่มีอะไรมายกเลิกต่อ มัน home สำเร็จจริง — เป็นทางเดียวในระบบตอนนี้ที่ lift
ได้ตั้งศูนย์ encoder ใหม่ (เพราะ RESET_STEP ทำไม่สำเร็จตามข้อ 1)
