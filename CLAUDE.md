# Project Overview
โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์อัตโนมัติ (Auto Robot - R2) ในการแข่งขัน ABU Robocon พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์ by.Ittichai Wachiraphiphatkun

# Hardware & Tech Stack
 - Master Controller: Teensy 4.1 (ARM Cortex-M7)
 - Slave Controller: STM32 Blackpill (F411CE)
 - Development Environment: VS Code + PlatformIO (Multi-Environment Setup)
 - Control Algorithm: PID Control, Mecanum Kinematics

# Coding Rules
 - folder include ใช้ .h
 - folder src ใช้ .cpp
 - เขียนโค้ดให้ง่ายมากกว่ากว่าสั้นไป
 - ห้ามลบโค้ดเดิมถ้าไม่เข้าใจหน้าที่ของมัน
 - ห้ามใช้คำสั่ง delay() เด็ดขาด ให้ใช้ millis() หรือ Timer แทน เพื่อไม่ให้ลูปควบคุม (PID/Mecanum) สะดุด

# Workflow 
ก่อนแก้โค้ดทำตามนี้
 1. อ่านไฟล์ที่เกี่ยวข้องก่อน
 2. อธิบายแผนการแบบสั้นๆ
 3. แก้เฉพาะไฟล์ที่จำเป็น
 4. ตรวจว่าโค้ดไม่กระทบกับส่วนอื่น
 5. สรุปสิ่งที่แก้หลังทำเสร็จ