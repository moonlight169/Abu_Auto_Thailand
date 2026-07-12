#include <Arduino.h>
#include "config_sensor.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }

  pinMode(L_SW_lift_fonttop_1, INPUT_PULLUP);
  pinMode(L_SW_armback_2, INPUT_PULLUP);
  pinMode(L_SW_lift_fontbottom_3, INPUT_PULLUP);
  pinMode(L_SW_armfont_4, INPUT_PULLUP);
  pinMode(L_SW_lift_backtop_5, INPUT_PULLUP);
  pinMode(L_SW_fontrobot_6, INPUT_PULLUP);
  pinMode(L_SW_lift_backbottom_7, INPUT_PULLUP);
}

void loop() {
  // อ่านค่าจากเซนเซอร์ (0 = ทำงาน/ติด, 1 = ไม่ทำงาน/ดับ)
  // *เนื่องจากผ่าน Optocoupler ค่าที่ได้มักจะเป็น Inverse (Active Low)*

  Serial.print(" || Limits -> ");
  Serial.print("1: "); Serial.print(digitalRead(L_SW_lift_fonttop_1));
  Serial.print(" | 2: "); Serial.print(digitalRead(L_SW_armback_2));
  Serial.print(" | 3: "); Serial.print(digitalRead(L_SW_lift_fontbottom_3));
  Serial.print(" | 4: "); Serial.print(digitalRead(L_SW_armfont_4));
  Serial.print(" | 5: "); Serial.print(digitalRead(L_SW_lift_backtop_5));
  Serial.print(" | 6: "); Serial.print(digitalRead(L_SW_fontrobot_6));
  Serial.print(" | 7: "); Serial.println(digitalRead(L_SW_lift_backbottom_7));

  delay(200);
}