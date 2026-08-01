#include "arm_control.h"

#include "arm_limits.h"
#include "encoder.h"
#include "master_link.h"
#include "motor.h"

ArmState state = IDLE;
uint32_t stateStartMs = 0;

bool bottomHomed = false;
bool topHomed = false;
bool bottomHolding = false;
bool topHolding = false;

float bottomTargetDeg = 0.0f;
float topTargetDeg = 0.0f;

int testPwm = 0;

static float bottomIntegral = 0.0f;
static float bottomPreviousError = 0.0f;
static float topIntegral = 0.0f;
static float topPreviousError = 0.0f;
static uint32_t lastControlMs = 0;

const __FlashStringHelper *stateName() {
  switch (state) {
    case IDLE:                return F("IDLE");
    case HOME_BOTTOM_RELEASE: return F("HOME_BOTTOM_RELEASE");
    case HOME_BOTTOM_FIRST:   return F("HOME_BOTTOM_FIRST");
    case HOME_BOTTOM_CONFIRM: return F("HOME_BOTTOM_CONFIRM");
    case HOME_TOP_FRONT:      return F("HOME_TOP_FRONT");
    case HOME_TOP_TO_READY:   return F("HOME_TOP_TO_READY");
    case BOTTOM_POSITION_MOVE:return F("BOTTOM_POSITION_MOVE");
    case TOP_POSITION_MOVE:   return F("TOP_POSITION_MOVE");
    case TEST_BOTTOM_PWM:     return F("TEST_BOTTOM_PWM");
    case TEST_TOP_PWM:        return F("TEST_TOP_PWM");
    case FAULT:               return F("FAULT");
  }
  return F("UNKNOWN");
}

float getTopPositionDeg() {
  return (float)readTopEncoder() / TOP_PULSE_PER_DEG;
}

float getBottomPositionDeg() {
  return (float)readBottomEncoder() / BOTTOM_PULSE_PER_DEG;
}

void stopAll() {
  bottomMotor(0);
  topMotor(0);
  state = IDLE;
  bottomHolding = false;
  topHolding = false;
  bottomIntegral = 0.0f;
  topIntegral = 0.0f;
  testPwm = 0;
  masterJob = MASTER_JOB_NONE;
  Serial.println(F("[STOP] All motors stopped"));
}

void enterFault(const __FlashStringHelper *message) {
  bottomMotor(0);
  topMotor(0);
  state = FAULT;
  bottomHolding = false;
  topHolding = false;
  Serial.print(F("[FAULT] "));
  Serial.println(message);
  if (activeSequence != 0) {
    char faultText[48];
    snprintf(faultText, sizeof(faultText), "MOVEMENT_FAULT");
    masterSendError(activeSequence, activeCommand, faultText, true);
  }
  masterJob = MASTER_JOB_NONE;
}

void startHome() {
  bottomMotor(0);
  topMotor(0);
  bottomHomed = false;
  topHomed = false;
  bottomHolding = false;
  topHolding = false;
  bottomIntegral = 0.0f;
  topIntegral = 0.0f;
  stateStartMs = millis();

  // If Bottom is already pressing FRONT, move away first. This proves that
  // the switch can release and prevents HOME from skipping straight to Top.
  if (bottomFrontLimit.stablePressed) {
    state = HOME_BOTTOM_RELEASE;
    bottomMotor(BOTTOM_HOME_RELEASE_PWM);
    Serial.println(F("[HOME] Step 0: releasing Bottom FRONT limit"));
  } else {
    state = HOME_BOTTOM_FIRST;
    bottomMotor(BOTTOM_HOME_PWM);
    Serial.println(F("[HOME] Step 1: moving Bottom to FRONT"));
  }
  if (activeSequence != 0) {
    Serial1.print(F("STATE,"));
    Serial1.print(activeSequence);
    Serial1.println(F(",HOME_BOTTOM"));
  }
  Serial.println(F("[HOME] Bottom must finish before Top is allowed to move"));
}

bool commandBottomPosition(float degrees) {
  if (!bottomHomed) {
    Serial.println(F("[BLOCK] Bottom is not homed. Send HOME first."));
    return false;
  }
  topMotor(0);
  degrees = constrain(degrees, BOTTOM_MIN_DEG, BOTTOM_MAX_DEG);
  bottomTargetDeg = degrees;
  bottomHolding = false;
  bottomIntegral = 0.0f;
  bottomPreviousError = 0.0f;
  lastControlMs = millis();
  stateStartMs = millis();
  state = BOTTOM_POSITION_MOVE;
  Serial.print(F("[BOTTOM] Target = "));
  Serial.print(bottomTargetDeg, 1);
  Serial.println(F(" deg"));
  return true;
}

bool commandTopPosition(float degrees) {
  if (!topHomed) {
    Serial.println(F("[BLOCK] Top is not homed. Send HOME first."));
    return false;
  }

  bottomMotor(0);
  degrees = constrain(degrees, TOP_MIN_DEG, TOP_MAX_DEG);
  topTargetDeg = degrees;
  topHolding = false;
  topIntegral = 0.0f;
  topPreviousError = 0.0f;
  lastControlMs = millis();
  stateStartMs = millis();
  state = TOP_POSITION_MOVE;

  Serial.print(F("[TOP] Target = "));
  Serial.print(topTargetDeg, 1);
  Serial.println(F(" deg"));
  return true;
}

void startBottomPwmTest(int pwm) {
  stopAll();
  pwm = constrain(pwm, -255, 255);
  if (pwm == 0) return;
  testPwm = pwm;
  stateStartMs = millis();
  state = TEST_BOTTOM_PWM;
  Serial.print(F("[PWM TEST] Bottom PWM="));
  Serial.println(testPwm);
}

void startTopPwmTest(int pwm) {
  stopAll();
  pwm = constrain(pwm, -255, 255);
  if (pwm == 0) return;
  testPwm = pwm;
  stateStartMs = millis();
  state = TEST_TOP_PWM;
  Serial.print(F("[PWM TEST] Top PWM="));
  Serial.println(testPwm);
}

void updateStateMachine() {
  const uint32_t now = millis();

  if (state != IDLE && state != FAULT &&
      !((state == TOP_POSITION_MOVE && topHolding) ||
        (state == BOTTOM_POSITION_MOVE && bottomHolding)) &&
      now - stateStartMs > MOVE_TIMEOUT_MS) {
    enterFault(F("Movement timeout"));
    return;
  }

  switch (state) {
    case HOME_BOTTOM_RELEASE:
      // Top is positively locked out throughout the complete Bottom home.
      topMotor(0);
      if (!bottomFrontLimit.stablePressed) {
        bottomMotor(0);
        state = HOME_BOTTOM_FIRST;
        stateStartMs = now;
        bottomMotor(BOTTOM_HOME_PWM);
        Serial.println(F("[HOME] Bottom FRONT released; seeking FRONT again"));
      } else {
        bottomMotor(BOTTOM_HOME_RELEASE_PWM);
      }
      break;

    case HOME_BOTTOM_FIRST:
      topMotor(0);
      if (bottomFrontLimit.stablePressed) {
        bottomMotor(0);
        state = HOME_BOTTOM_CONFIRM;
        stateStartMs = now;
        if (activeSequence != 0) {
          Serial1.print(F("STATE,"));
          Serial1.print(activeSequence);
          Serial1.println(F(",HOME_BOTTOM_CONFIRM"));
        }
        Serial.println(F("[HOME] Bottom FRONT detected; confirming"));
      } else {
        bottomMotor(BOTTOM_HOME_PWM);
      }
      break;

    case HOME_BOTTOM_CONFIRM:
      // Both motors stay stopped while the Bottom limit is confirmed.
      bottomMotor(0);
      topMotor(0);
      if (!bottomFrontLimit.stablePressed) {
        state = HOME_BOTTOM_FIRST;
        stateStartMs = now;
        bottomMotor(BOTTOM_HOME_PWM);
        Serial.println(F("[HOME] Bottom limit released; retrying Bottom"));
      } else if (now - stateStartMs >= BOTTOM_HOME_CONFIRM_MS) {
        resetBottomEncoder();
        bottomHomed = true;
        state = HOME_TOP_FRONT;
        stateStartMs = now;
        topMotor(TOP_HOME_PWM);
        if (activeSequence != 0) {
          Serial1.print(F("STATE,"));
          Serial1.print(activeSequence);
          Serial1.println(F(",HOME_TOP"));
        }
        Serial.println(F("[HOME] Step 1 complete: Bottom homed"));
        Serial.println(F("[HOME] Step 2: moving Top to FRONT"));
      }
      break;

    case BOTTOM_POSITION_MOVE: {
      if (now - lastControlMs < CONTROL_PERIOD_MS) break;
      float dt = (now - lastControlMs) / 1000.0f;
      lastControlMs = now;
      const float error =
          bottomTargetDeg * BOTTOM_PULSE_PER_DEG - (float)readBottomEncoder();
      const float tolerancePulse =
          BOTTOM_TOLERANCE_DEG * fabs(BOTTOM_PULSE_PER_DEG);
      const float releasePulse =
          BOTTOM_RELEASE_DEG * fabs(BOTTOM_PULSE_PER_DEG);

      if (!bottomHolding && fabs(error) <= tolerancePulse) {
        bottomHolding = true;
      } else if (bottomHolding && fabs(error) >= releasePulse) {
        bottomHolding = false;
      }
      if (bottomHolding) {
        bottomMotor(0);
        break;
      }

      bottomIntegral += error * dt;
      bottomIntegral = constrain(bottomIntegral, -20000.0f, 20000.0f);
      const float derivative =
          dt > 0.0f ? (error - bottomPreviousError) / dt : 0.0f;
      bottomPreviousError = error;
      int pwm = (int)(BOTTOM_KP * error +
                      BOTTOM_KI * bottomIntegral +
                      BOTTOM_KD * derivative);
      pwm = constrain(pwm, -BOTTOM_MAX_PWM, BOTTOM_MAX_PWM);
      if (pwm != 0 && abs(pwm) < BOTTOM_MIN_MOVE_PWM) {
        pwm = pwm > 0 ? BOTTOM_MIN_MOVE_PWM : -BOTTOM_MIN_MOVE_PWM;
      }

  if (pwm < 0 && bottomFrontLimit.stablePressed)
  {
    bottomMotor(0);

    const float currentDeg = getBottomPositionDeg();

    // ยอมรับลิมิต FRONT เฉพาะตอนอยู่ใกล้ตำแหน่ง Home จริง
    if (bottomTargetDeg <= BOTTOM_TOLERANCE_DEG &&
        fabs(currentDeg) <= 5.0f)
    {
      resetBottomEncoder();
      bottomHolding = true;
      bottomIntegral = 0.0f;

      Serial.println(F("[BOTTOM] FRONT reached: position reset to 0"));
    }
    else
    {
      // ไม่รีเซ็ต Encoder เพราะอาจเป็นลิมิตค้างหรือสายผิด
      enterFault(F("BOTTOM FRONT active away from home"));
    }
  } else if (pwm > 0 && isPressed(BOTTOM_LIMIT_BACK)) {
        bottomMotor(0);
        state = IDLE;
        Serial.println(F("[BOTTOM] BACK limit reached before target"));
      } else {
        bottomMotor(pwm);
      }
      break;
    }

    case HOME_TOP_FRONT:
      if (topFrontLimit.stablePressed) {
        topMotor(0);
        resetTopEncoder();
        topTargetDeg = TOP_HOME_READY_DEG;
        topHolding = false;
        topIntegral = 0.0f;
        topPreviousError = 0.0f;
        lastControlMs = now;
        stateStartMs = now;
        state = HOME_TOP_TO_READY;
        if (activeSequence != 0) {
          Serial1.print(F("STATE,"));
          Serial1.print(activeSequence);
          Serial1.println(F(",TOP_TO_READY"));
        }
        Serial.println(F("[HOME] Step 2 complete: Top FRONT = 0.0 deg"));
        Serial.print(F("[HOME] Step 3: moving Top to "));
        Serial.print(TOP_HOME_READY_DEG, 1);
        Serial.println(F(" deg"));
      }
      break;

    case HOME_TOP_TO_READY: {
      if (now - lastControlMs < CONTROL_PERIOD_MS) break;

      const float dt = (now - lastControlMs) / 1000.0f;
      lastControlMs = now;
      const float targetPulse = TOP_HOME_READY_DEG * TOP_PULSE_PER_DEG;
      const float currentPulse = (float)readTopEncoder();
      const float error = targetPulse - currentPulse;
      const float tolerancePulse = TOP_TOLERANCE_DEG * fabs(TOP_PULSE_PER_DEG);

      if (fabs(error) <= tolerancePulse) {
        topMotor(0);
        topTargetDeg = TOP_HOME_READY_DEG;
        topHolding = true;
        topHomed = true;
        state = TOP_POSITION_MOVE;
        stateStartMs = now;
        Serial.print(F("[HOME] Complete: Bottom FRONT = 0.0 deg, Top = "));
        Serial.print(TOP_HOME_READY_DEG, 1);
        Serial.println(F(" deg"));
        break;
      }

      topIntegral += error * dt;
      topIntegral = constrain(topIntegral, -20000.0f, 20000.0f);
      const float derivative =
          dt > 0.0f ? (error - topPreviousError) / dt : 0.0f;
      topPreviousError = error;

      int pwm = (int)(TOP_KP * error +
                      TOP_KI * topIntegral +
                      TOP_KD * derivative);
      pwm = constrain(pwm, -TOP_MAX_PWM, TOP_MAX_PWM);
      if (pwm != 0 && abs(pwm) < TOP_MIN_MOVE_PWM) {
        pwm = pwm > 0 ? TOP_MIN_MOVE_PWM : -TOP_MIN_MOVE_PWM;
      }

      if (pwm < 0 && isPressed(TOP_LIMIT_BACK)) {
        enterFault(F("TOP BACK limit reached during HOME"));
      } else if (pwm > 0 && isPressed(TOP_LIMIT_FRONT)) {
        enterFault(F("Top moved toward FRONT after zero"));
      } else {
        topMotor(pwm);
      }
      break;
    }

    case TOP_POSITION_MOVE: {
      if (now - lastControlMs < CONTROL_PERIOD_MS) break;

      float dt = (now - lastControlMs) / 1000.0f;
      lastControlMs = now;
      const float targetPulse = topTargetDeg * TOP_PULSE_PER_DEG;
      const float currentPulse = (float)readTopEncoder();
      const float error = targetPulse - currentPulse;
      const float tolerancePulse = TOP_TOLERANCE_DEG * fabs(TOP_PULSE_PER_DEG);
      const float releasePulse = TOP_RELEASE_DEG * fabs(TOP_PULSE_PER_DEG);

      if (!topHolding && fabs(error) <= tolerancePulse) {
        topHolding = true;
      } else if (topHolding && fabs(error) >= releasePulse) {
        topHolding = false;
      }

      if (topHolding) {
        topMotor(0);
        break;
      }

      topIntegral += error * dt;
      topIntegral = constrain(topIntegral, -20000.0f, 20000.0f);
      const float derivative = dt > 0.0f ? (error - topPreviousError) / dt : 0.0f;
      topPreviousError = error;

      // TOP_PULSE_PER_DEG is negative, while positive motor PWM goes FRONT.
      // Therefore controller output is inverted before applying to the motor.
      float controller = TOP_KP * error +
                         TOP_KI * topIntegral +
                         TOP_KD * derivative;
      int pwm = (int)controller;
      pwm = constrain(pwm, -TOP_MAX_PWM, TOP_MAX_PWM);

      if (pwm != 0 && abs(pwm) < TOP_MIN_MOVE_PWM) {
        pwm = pwm > 0 ? TOP_MIN_MOVE_PWM : -TOP_MIN_MOVE_PWM;
      }

      // The back limit is a hard end stop. Its actual angle may differ from
      // TOP_MAX_DEG, so reaching it stops motion safely.
      if (pwm < 0 && isPressed(TOP_LIMIT_BACK)) {
        topMotor(0);
        state = IDLE;
        Serial.println(F("[TOP] BACK limit reached before target"));
      } else if (pwm > 0 && isPressed(TOP_LIMIT_FRONT)) {
        topMotor(0);
        resetTopEncoder();  // refresh the physical zero reference
        if (topTargetDeg <= TOP_TOLERANCE_DEG) {
          topHolding = true;
        } else {
          enterFault(F("Unexpected TOP FRONT limit"));
        }
      } else {
        topMotor(pwm);
      }
      break;
    }

    case TEST_BOTTOM_PWM:
      // Re-check the directional limit every loop during manual PWM testing.
      if ((testPwm < 0 && isPressed(BOTTOM_LIMIT_FRONT)) ||
          (testPwm > 0 && isPressed(BOTTOM_LIMIT_BACK))) {
        bottomMotor(0);
        testPwm = 0;
        state = IDLE;
        Serial.println(F("[PWM TEST] Bottom stopped by directional limit"));
      } else {
        bottomMotor(testPwm);
      }
      break;

    case TEST_TOP_PWM:
      // Positive Top PWM = FRONT, negative Top PWM = BACK.
      if ((testPwm > 0 && isPressed(TOP_LIMIT_FRONT)) ||
          (testPwm < 0 && isPressed(TOP_LIMIT_BACK))) {
        topMotor(0);
        testPwm = 0;
        state = IDLE;
        Serial.println(F("[PWM TEST] Top stopped by directional limit"));
      } else {
        topMotor(testPwm);
      }
      break;

    case IDLE:
    case FAULT:
      break;
  }
}
