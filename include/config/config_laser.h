//Active LOW
#define L_SW_frontrobot_6 PB1

// ย้ายจาก PB6/PB7 เดิม: บอร์ด Blackpill F411CE ไม่มี ADC บนขา PB6/PB7 (analogRead ใช้ไม่ได้จริง)
// PA1, PB0 เป็นขา ADC1 ที่ว่างจริง ไม่ชนกับปุ่ม USER_BTN(PA0), UART(PA9/PA10), USB(PA11/PA12), SWD(PA13/PA14)
#define Light_sensor_1 PA1
#define Light_sensor_2 PB0

//Active LOW (สมมติฐานตาม convention เดิมของโปรเจกต์ — ต้องเช็คกับสเปก sensor laser จริงก่อนต่อสาย)
// พินแนะนำ (ยังไม่ได้ยืนยันกับวงจรจริง) — เลือกขาที่ว่าง ไม่ชน BOOT1(PB2), UART, USB, SWD
#define Laser_1 PB10
#define Laser_2 PB12
#define Laser_3 PB13
#define Laser_4 PB14
#define Laser_5 PB15
