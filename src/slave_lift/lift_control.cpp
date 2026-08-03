#include "lift_control.h"

#include <stdlib.h>

#include "encoder.h"
#include "master_link.h"
#include "motor.h"

int32_t targetFront = 0;
int32_t targetBack = 0;
float setpointFront = 0.0f;
float setpointBack = 0.0f;
bool pidEnabled = false;
bool homing = false;
int frontPWM = 0;
int backPWM = 0;

static PositionPID pidFront;
static PositionPID pidBack;

static bool frontHomed = false;
static bool backHomed = false;
static uint16_t reachedCounter = 0;
static bool reachedReported = false;
static uint32_t lastControlTime = 0;

void resetControllers() {
  pidFront.reset();
  pidBack.reset();
}

void stopMotors() {
  pidEnabled = false;
  homing = false;
  reachedCounter = 0;
  reachedReported = false;
  frontPWM = 0;
  backPWM = 0;
  resetControllers();
  frontMotor(0);
  backMotor(0);
}

// ---------------------------------------- home and position control -------
void beginHome() {
  stopMotors();
  homing = true;
  frontHomed = false;
  backHomed = false;
  sendMasterStatus("LIFT_HOMING");
  Serial.println(F("HOME START"));
}

void setTarget(int32_t frontTarget, int32_t backTarget) {
  const int32_t newFront =
      constrain(frontTarget, 0L, (long)FRONT_PULSE_MAX);
  const int32_t newBack =
      constrain(backTarget, 0L, (long)BACK_PULSE_MAX);

  // Do not reset the PID if the Master accidentally repeats the same command.
  if (pidEnabled && newFront == targetFront && newBack == targetBack) {
    return;
  }

  targetFront = newFront;
  targetBack = newBack;

  setpointFront = (float)targetFront;
  setpointBack = (float)targetBack;

  resetControllers();
  reachedCounter = 0;
  reachedReported = false;
  lastControlTime = millis();
  pidEnabled = true;
  homing = false;
  sendMasterStatus("LIFT_BUSY");

  Serial.print(F("TARGET F="));
  Serial.print(targetFront);
  Serial.print(F(" B="));
  Serial.print(targetBack);
  Serial.println(F(" | POSITION PID"));
}

void zeroEncodersAndTargets() {
  stopMotors();
  resetBothEncoders();
  targetFront = 0;
  targetBack = 0;
  setpointFront = 0.0f;
  setpointBack = 0.0f;
}

void updateControl() {
  const uint32_t now = millis();
  if ((uint32_t)(now - lastControlTime) < CONTROL_PERIOD_MS) return;

  const float dt = (lastControlTime == 0)
                       ? CONTROL_PERIOD_MS * 0.001f
                       : (now - lastControlTime) * 0.001f;
  lastControlTime = now;

  if (homing) {
    if (limitActive(FRONT_LIMIT_BOTTOM)) {
      frontPWM = 0;
      frontMotor(0);
      if (!frontHomed) {
        resetFrontEncoder();
        frontHomed = true;
        Serial.println(F("HOME FRONT OK"));
      }
    } else {
      frontPWM = HOME_PWM;
      frontMotor(frontPWM);
    }

    if (limitActive(BACK_LIMIT_BOTTOM)) {
      backPWM = 0;
      backMotor(0);
      if (!backHomed) {
        resetBackEncoder();
        backHomed = true;
        Serial.println(F("HOME BACK OK"));
      }
    } else {
      backPWM = HOME_PWM;
      backMotor(backPWM);
    }

    if (frontHomed && backHomed) {
      homing = false;
      targetFront = 0;
      targetBack = 0;
      setpointFront = 0.0f;
      setpointBack = 0.0f;
      sendMasterStatus("LIFT_HOME_REACHED");
      Serial.println(F("HOME COMPLETE"));
    }
    return;
  }

  if (!pidEnabled) {
    frontMotor(0);
    backMotor(0);
    return;
  }

  const int32_t front = readFrontEncoder();
  const int32_t back = readBackEncoder();

  const int frontTargetPWM = calculatePositionPWM(
      pidFront, setpointFront, front, dt,
      frontKp, frontKi, frontKd);
  const int backTargetPWM = calculatePositionPWM(
      pidBack, setpointBack, back, dt,
      backKp, backKi, backKd);

  // Positive PWM is downward. Ramp up gently and reduce the PWM limit
  // progressively during the last DOWN_BRAKE_ZONE_PULSE pulses.
  frontPWM = applyDownSoftPWM(
      frontTargetPWM, frontPWM, targetFront, front,
      FRONT_DOWN_PWM_MAX);
  backPWM = applyDownSoftPWM(
      backTargetPWM, backPWM, targetBack, back,
      BACK_DOWN_PWM_MAX);

  // Limit cuts only the direction moving into that switch.
  // The opposite direction remains available to leave the switch.
  if (frontPWM < 0 && limitActive(FRONT_LIMIT_TOP)) {
    frontPWM = 0;
    targetFront = front;
    setpointFront = (float)front;
    pidFront.reset();
    Serial.println(F("LIMIT FRONT TOP"));
  } else if (frontPWM > 0 && limitActive(FRONT_LIMIT_BOTTOM)) {
    frontPWM = 0;
    resetFrontEncoder();
    targetFront = 0;
    setpointFront = 0.0f;
    pidFront.reset();
    Serial.println(F("LIMIT FRONT BOTTOM"));
  }

  if (backPWM < 0 && limitActive(BACK_LIMIT_TOP)) {
    backPWM = 0;
    targetBack = back;
    setpointBack = (float)back;
    pidBack.reset();
    Serial.println(F("LIMIT BACK TOP"));
  } else if (backPWM > 0 && limitActive(BACK_LIMIT_BOTTOM)) {
    backPWM = 0;
    resetBackEncoder();
    targetBack = 0;
    setpointBack = 0.0f;
    pidBack.reset();
    Serial.println(F("LIMIT BACK BOTTOM"));
  }

  frontMotor(frontPWM);
  backMotor(backPWM);

  // Both axes must remain inside the tolerance for 200 ms.
  const int32_t checkedFront = readFrontEncoder();
  const int32_t checkedBack = readBackEncoder();
  const bool frontReached =
      labs(targetFront - checkedFront) <= POSITION_TOLERANCE_PULSE;
  const bool backReached =
      labs(targetBack - checkedBack) <= POSITION_TOLERANCE_PULSE;

  if (frontReached && backReached) {
    if (reachedCounter < REACHED_CONFIRM_CYCLES) {
      reachedCounter++;
    }

    if (reachedCounter >= REACHED_CONFIRM_CYCLES &&
        !reachedReported) {
      reachedReported = true;
      sendLiftReached(checkedFront, checkedBack);
    }
  } else {
    reachedCounter = 0;
    reachedReported = false;
  }
}
