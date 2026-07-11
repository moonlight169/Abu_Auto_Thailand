# Project Overview
โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์อัตโนมัติ (Auto Robot - R2) ในการแข่งขัน ABU Robocon 
พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์ by.Ittichai Wachiraphiphatkun

# Hardware & Tech Stack
- Master Controller: Teensy 4.1 (ARM Cortex-M7)
- Slave Controller: STM32 Blackpill (F411CE)
- Development Environment: VS Code + PlatformIO (Multi-Environment Setup)
- Control Algorithm: PID Control, Mecanum Kinematics

# Project Structure
ระบบเป็น Distributed System เชื่อมต่อแบบ Star Topology ผ่านสาย UART (JST-XH 4 pin) แยกอิสระต่อบอร์ด (Point-to-Point) 
โดย Slave ทุกตัวจะต่อตรงเข้าหา Master โดยตรง (Slave 1 -> Master, Slave 2 -> Master, Slave 3 -> Master, Slave 4 -> Master)
ประกอบด้วย 5 โฟลเดอร์แยก Environment ชัดเจน:
├── 0_master/       # Teensy 4.1: ศูนย์กลางสั่งการ (มี Hardware Serial 4 ช่อง แยกคุยกับแต่ละ Slave)
├── 1_slave_wheel/  # STM32: คุมล้อ Mecanum 4 ล้อ (ลูป PID ความถี่สูง)
├── 2_slave_lift/   # STM32: คุมชุดยกโซ่เฟือง 2 ตัว + ชุดแขน 2 ตัว
├── 3_slave_sensor/ # STM32: คุม Relay 4 ตัว + Servo 2 ตัว + อ่าน Limit Switch 7 ตัว
└── 4_slave_laser/  # STM32: อ่าน Laser Check Field 5 ตัว (Active Low) + TOF 3 ตัว (UART)

# Coding Rules (กฎเหล็ก)
1. **File Extension:** โฟลเดอร์ `include/` ใช้ `.h` เท่านั้น, โฟลเดอร์ `src/` ใช้ `.cpp` เท่านั้น
2. **Readability:** เขียนโค้ดให้เข้าใจและไล่ง่ายมากกว่าเน้นโค้ดสั้น ห้ามยัดคำสั่งในบรรทัดเดียวจนอ่านยาก
3. **Legacy Code:** ห้ามลบหรือเปลี่ยนโค้ดเดิมเด็ดขาด หากยังไม่เข้าใจหน้าที่และผลกระทบของโค้ดนั้น
4. **No delay():** ห้ามใช้คำสั่ง `delay()` ในบอร์ดใดๆ ทั้งสิ้น ให้ใช้ `millis()` หรือโปรแกรมโครงสร้าง Timer/Non-blocking เท่านั้น เพื่อไม่ให้ลูปควบคุม (PID/Mecanum) สะดุด

# AI Workflow (ลำดับขั้นตอนทำงานของ Claude Code)
ก่อนทำการแก้ไขหรือแนะนำโค้ด ให้เดินตามลูป 6 ขั้นตอนนี้เสมอ:
1. **Analyze:** อ่านและวิเคราะห์ไฟล์ที่เกี่ยวข้องใน Environment นั้นๆ 
2. **Plan:** อธิบายแผนการปรับปรุงสั้นๆ เป็นข้อๆ ให้ผู้ใช้ฟังสรุปก่อน
3. **Impact Check:** ประเมินผลกระทบว่ากระทบต่อลูป PID, ไทม์มิ่ง หรือการสื่อสารระหว่างบอร์ดหรือไม่
4. **Guide & Teach (CRITICAL):** อธิบายวิธีเขียน ห้ามแอบเขียนโค้ดสำเร็จรูปลงไฟล์เด็ดขาด ให้ผู้ใช้เป็นคนพิมพ์โค้ดเองทั้งหมด (ยกเว้นมีคำสั่งเจาะจงให้เขียนแทน)
5. **Summary:** สรุปสิ่งที่ได้แก้ไข และข้อควรระวังหลังจากทำเสร็จ
6. **Git Commit:** แนะนำข้อความสำหรับการ `git commit` ตามมาตรฐาน Conventional Commits (เช่น feat(wheel): ..., fix(sensor): ...)