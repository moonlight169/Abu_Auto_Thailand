#include <Arduino.h>
#include "lift.h"

const int LIFT_HOME_SPEED = 60;

const long LIFT_PULSE_TOLERANCE = 20;

const float LIFT_PID_KP_FRONT = 1.0;
const float LIFT_PID_KI_FRONT = 0.05;
const float LIFT_PID_KD_FRONT = 0.00;

const float LIFT_PID_KP_BACK = 0.7;
const float LIFT_PID_KI_BACK = 0.03;
const float LIFT_PID_KD_BACK = 0.00;

const int LIFT_PID_OUT_MIN = -255;
const int LIFT_PID_OUT_MAX = 255;

//ช่วง pulse จริงที่วัดได้จากฮาร์ดแวร์ (อ้างอิงคอมเมนต์ใน config_lift.h)
const long LIFT_FRONT_PULSE_MIN = 0;
const long LIFT_FRONT_PULSE_MAX = 4500;
const long LIFT_BACK_PULSE_MIN = 0;
const long LIFT_BACK_PULSE_MAX = 4100;

//TODO: วัดระยะชักจริงจาก home ถึง top ของแต่ละฝั่ง (mm) ก่อนใช้ liftToMM()
const float LIFT_STROKE_MM_FRONT = 0.0;
const float LIFT_STROKE_MM_BACK = 0.0;

//TODO: tune - gain synchronize ให้ RPM front/back เท่ากันตอนสั่ง target เท่ากัน เริ่มจาก 0 แล้วค่อยไล่ขึ้น
const float LIFT_SYNC_KP = 0.0;

//TODO: tune - PWM อ่อนสุดที่ยังไหลลงจนแตะ limit switch ล่างได้ ใช้แทน PID ตอน target = 0
const int LIFT_HOLD_PWM = 40;

Lift::Lift(int frontMotorA, int frontMotorB, int backMotorA, int backMotorB,
           int encFrontA, int encFrontB, int encBackA, int encBackB,
           int swFrontTop, int swFrontBottom, int swBackTop, int swBackBottom)
    : _motorFront(frontMotorA, frontMotorB),
      _motorBack(backMotorA, backMotorB),
      _encoderFront(encFrontA, encFrontB, 1.0),
      _encoderBack(encBackA, encBackB, 1.0),
      _pidFront(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX, LIFT_PID_KP_FRONT, LIFT_PID_KI_FRONT, LIFT_PID_KD_FRONT),
      _pidBack(LIFT_PID_OUT_MIN, LIFT_PID_OUT_MAX, LIFT_PID_KP_BACK, LIFT_PID_KI_BACK, LIFT_PID_KD_BACK){

    this->_swFrontTop = swFrontTop;
    this->_swFrontBottom = swFrontBottom;
    this->_swBackTop = swBackTop;
    this->_swBackBottom = swBackBottom;

    pinMode(swFrontTop, INPUT_PULLUP);
    pinMode(swFrontBottom, INPUT_PULLUP);
    pinMode(swBackTop, INPUT_PULLUP);
    pinMode(swBackBottom, INPUT_PULLUP);
}

void Lift::setZero(){
    this->_state = LIFT_HOMING;
    this->_frontHomed = false;
    this->_backHomed = false;
}

void Lift::liftTo(long pulseFront, long pulseBack){
    this->_targetPulseFront = constrain(pulseFront, LIFT_FRONT_PULSE_MIN, LIFT_FRONT_PULSE_MAX);
    this->_targetPulseBack = constrain(pulseBack, LIFT_BACK_PULSE_MIN, LIFT_BACK_PULSE_MAX);
    this->_state = LIFT_MOVING;
}

void Lift::liftToMM(float mmFront, float mmBack){
    long pulseFront = map((long)mmFront, 0, (long)LIFT_STROKE_MM_FRONT, LIFT_FRONT_PULSE_MIN, LIFT_FRONT_PULSE_MAX);
    long pulseBack = map((long)mmBack, 0, (long)LIFT_STROKE_MM_BACK, LIFT_BACK_PULSE_MIN, LIFT_BACK_PULSE_MAX);
    this->liftTo(pulseFront, pulseBack);
}

void Lift::stop(){
    this->_state = LIFT_IDLE;
    this->_motorFront.run(0);
    this->_motorBack.run(0);
}

void Lift::applySafetyCutoff(){
    bool frontTop = (digitalRead(this->_swFrontTop) == LOW);
    bool frontBottom = (digitalRead(this->_swFrontBottom) == LOW);
    bool backTop = (digitalRead(this->_swBackTop) == LOW);
    bool backBottom = (digitalRead(this->_swBackBottom) == LOW);

    if (frontTop && this->_motorFront.getSpeed() < 0){
        this->_motorFront.run(0);
    }
    if (frontBottom && this->_motorFront.getSpeed() > 0){
        this->_motorFront.run(0);
    }
    if (backTop && this->_motorBack.getSpeed() < 0){
        this->_motorBack.run(0);
    }
    if (backBottom && this->_motorBack.getSpeed() > 0){
        this->_motorBack.run(0);
    }

    //ชนขา limit ล่าง = อยู่ตำแหน่ง home จริง เคลียร์ count กลับ 0 ทันที ทุก state (ไม่ใช่แค่ตอน homing) กัน count drift สะสม
    if (frontBottom){
        this->_encoderFront.reset();
    }
    if (backBottom){
        this->_encoderBack.reset();
    }
}

void Lift::update(){
    if (this->_state == LIFT_HOMING){
        bool frontBottom = (digitalRead(this->_swFrontBottom) == LOW);
        bool backBottom = (digitalRead(this->_swBackBottom) == LOW);

        if (!frontBottom){
            this->_motorFront.run(LIFT_HOME_SPEED);
        } else {
            this->_motorFront.run(0);
            if (!this->_frontHomed){
                this->_encoderFront.reset();
                this->_frontHomed = true;
            }
        }

        if (!backBottom){
            this->_motorBack.run(LIFT_HOME_SPEED);
        } else {
            this->_motorBack.run(0);
            if (!this->_backHomed){
                this->_encoderBack.reset();
                this->_backHomed = true;
            }
        }

        if (this->_frontHomed && this->_backHomed){
            this->_state = LIFT_IDLE;
        }
    }
    else if (this->_state == LIFT_MOVING){
        //error บวก (ต้องการ count เพิ่ม = ต้องขึ้น) ต้องสั่ง speed ลบ ตาม convention เดิมของฮาร์ดแวร์ตัวนี้
        bool frontAtZero = (this->_targetPulseFront == 0);
        bool backAtZero = (this->_targetPulseBack == 0);

        int frontOutput;
        if (frontAtZero){
            //target 0 = จอดชิด limit ล่าง เลี้ยง pwm อ่อนๆ แทน PID เต็มรูป กันสั่นตอนใกล้ปลายทาง ปล่อยให้ limit switch เป็นตัวหยุดจริง
            frontOutput = LIFT_HOLD_PWM;
        } else {
            frontOutput = -this->_pidFront.compute(this->_targetPulseFront, this->_encoderFront.getCount());
        }

        int backOutput;
        if (backAtZero){
            backOutput = LIFT_HOLD_PWM;
        } else {
            backOutput = -this->_pidBack.compute(this->_targetPulseBack, this->_encoderBack.getCount());
        }

        //balance รอบ RPM เมื่อสั่ง target หน้า/หลังเท่ากัน (ข้ามตอนจอดที่ 0 เพราะเลี้ยง pwm คงที่เท่ากันอยู่แล้ว)
        if (!frontAtZero && !backAtZero && this->_targetPulseFront == this->_targetPulseBack){
            float rpmError = this->_encoderFront.getRPM() - this->_encoderBack.getRPM();
            int syncCorrection = (int)(LIFT_SYNC_KP * rpmError);
            frontOutput += syncCorrection;
            backOutput -= syncCorrection;
        }

        this->_motorFront.run(frontOutput);
        this->_motorBack.run(backOutput);
    }

    this->applySafetyCutoff();
}

void Lift::handleFrontA(){
    this->_encoderFront.handleA();
}

void Lift::handleFrontB(){
    this->_encoderFront.handleB();
}

void Lift::handleBackA(){
    this->_encoderBack.handleA();
}

void Lift::handleBackB(){
    this->_encoderBack.handleB();
}

long Lift::getFrontCount(){
    return this->_encoderFront.getCount();
}

long Lift::getBackCount(){
    return this->_encoderBack.getCount();
}

void Lift::CountDebug(){
    Serial.print("Lift Count -> Front: ");
    Serial.print(this->getFrontCount());
    Serial.print(" | Back: ");
    Serial.println(this->getBackCount());
}

void Lift::PWMDebug(){
    Serial.print("Lift PWM -> Front: ");
    Serial.print(this->_motorFront.getSpeed());
    Serial.print(" | Back: ");
Serial.println(this->_motorBack.getSpeed());
}

bool Lift::isBusy(){
    return this->_state != LIFT_IDLE;
}

bool Lift::atTarget(){
    long frontError = abs(this->_targetPulseFront - this->_encoderFront.getCount());
    long backError = abs(this->_targetPulseBack - this->_encoderBack.getCount());

    return (frontError <= LIFT_PULSE_TOLERANCE) && (backError <= LIFT_PULSE_TOLERANCE);
}