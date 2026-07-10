# Abu_Auto_Thailand

โปรเจกต์ซอร์สโค้ดสำหรับควบคุมหุ่นยนต์อัตโนมัติ (Auto Robot - R2)
ในการแข่งขัน ABU Robocon พัฒนาโดยทีม KTC_DINO_ROBOT วิทยาลัยเทคนิคกาฬสินธุ์
by.Ittichai Wachiraphiphatkun

[![PlatformIO](https://img.shields.io/badge/PlatformIO-orange?style=flat&logo=platformio)](https://platformio.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=c%2B%2B)](https://isocpp.org/) [![Teensy](https://img.shields.io/badge/Teensy-4.1-green?style=flat)](https://www.pjrc.com/teensy/) [![STM32](https://img.shields.io/badge/STM32-F411CE-blue?style=flat&logo=stmicroelectronics)](https://www.st.com/)


## System Architecture
เนื่องจากหุ่นยนต์อัตโนมัติต้องการการคำนวณขั้นสูง (เช่น Odometry, Kinematics และ Path Tracking) โปรเจกต์นี้จึงเลือกใช้ไมโครคอนโทรลเลอร์ประสิทธิภาพสูงในการจัดการระบบแบบ Master-Slave เพื่อไม่ให้เกิดคอขวดในการประมวลผล

## 🛠️Hardware & Tech Stack
-   **Master Controller:** Teensy 4.1 (ARM Cortex-M7)
-   **Slave Controller:** STM32 Blackpill (F411CE)
-   **Development Environment:** VS Code + PlatformIO (Multi-Environment Setup)
-   **Control Algorithm:** PID Control, Mecanum Kinematics, Path Tracking

## 📂 Project Structure
```text
src/
 ├── master/         # โค้ดสำหรับ Teensy 4.1 (ประมวลผลเส้นทาง & ระบบนำทางอัตโนมัติ)
 └── slave_wheels/   # โค้ดสำหรับ STM32 (ควบคุมความเร็วมอเตอร์ล้อ & อ่านค่า Encoder)``
```
