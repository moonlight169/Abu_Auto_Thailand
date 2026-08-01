#include "high_align.h"

#include <math.h>

#include "box_parking.h"
#include "box_result.h"
#include "gyro.h"
#include "hub_link.h"
#include "kinematics.h"
#include "lift_link.h"
#include "mission_runner.h"
#include "position_control.h"
#include "robot_state.h"
#include "tasks.h"
#include "wheel_link.h"

bool higherAlignActive = false;

static HigherAlignState higherAlignState = HIGH_ALIGN_IDLE;

static float higherAlignTargetYawDeg = 0.0f;
static int32_t higherAfterLiftFrontPulse =
    HIGH_DEFAULT_AFTER_LIFT_FRONT_PULSE;
static int32_t higherAfterLiftBackPulse =
    HIGH_DEFAULT_AFTER_LIFT_BACK_PULSE;
static uint32_t higherAlignStartMs = 0;
static uint32_t higherAlignBothSinceMs = 0;
static uint32_t higherAlignBackoffStartMs = 0;

static bool higherSoftLiftActive = false;
static int32_t higherSoftLiftCommandFront = 0;
static int32_t higherSoftLiftCommandBack = 0;
static int32_t higherSoftLiftTargetFront = 0;
static int32_t higherSoftLiftTargetBack = 0;
static uint32_t higherSoftLiftLastStepMs = 0;
static uint32_t higherSoftLiftStartMs = 0;

static int32_t movePulseToward(int32_t current, int32_t target, int32_t step)
{
  if (current < target)
    return min(current + step, target);
  if (current > target)
    return max(current - step, target);
  return target;
}

static void startHigherSoftLift(int32_t targetFront, int32_t targetBack)
{
  higherSoftLiftTargetFront = targetFront;
  higherSoftLiftTargetBack = targetBack;
  higherSoftLiftCommandFront = liftPosFront;
  higherSoftLiftCommandBack = liftPosBack;
  higherSoftLiftLastStepMs = millis() - HIGH_SOFT_LIFT_STEP_MS;
  higherSoftLiftStartMs = millis();
  higherSoftLiftActive = true;
  liftTaskStatus = TASK_RUNNING;

  Serial.print("HIGH SOFT LIFT START: current LP,");
  Serial.print(higherSoftLiftCommandFront);
  Serial.print(',');
  Serial.print(higherSoftLiftCommandBack);
  Serial.print(" target LP,");
  Serial.print(targetFront);
  Serial.print(',');
  Serial.println(targetBack);
}

static void updateHigherSoftLift()
{
  if (!higherSoftLiftActive)
    return;

  const uint32_t now = millis();
  if ((uint32_t)(now - higherSoftLiftStartMs) >
      HIGH_LIFT_TIMEOUT_MS)
  {
    higherSoftLiftActive = false;
    liftTaskStatus = TASK_TIMEOUT;
    return;
  }

  if ((uint32_t)(now - higherSoftLiftLastStepMs) <
      HIGH_SOFT_LIFT_STEP_MS)
    return;

  higherSoftLiftCommandFront =
      movePulseToward(higherSoftLiftCommandFront,
                      higherSoftLiftTargetFront,
                      HIGH_SOFT_LIFT_STEP_PULSE);
  higherSoftLiftCommandBack =
      movePulseToward(higherSoftLiftCommandBack,
                      higherSoftLiftTargetBack,
                      HIGH_SOFT_LIFT_STEP_PULSE);

  startLift(higherSoftLiftCommandFront,
            higherSoftLiftCommandBack,
            HIGH_LIFT_TIMEOUT_MS);
  higherSoftLiftLastStepMs = now;

  if (higherSoftLiftCommandFront == higherSoftLiftTargetFront &&
      higherSoftLiftCommandBack == higherSoftLiftTargetBack)
  {
    higherSoftLiftActive = false;
    Serial.println("HIGH SOFT LIFT: FINAL TARGET SENT");
  }
}

static void driveHigherStepContact()
{
  const float yawErrorDeg = normalizeAngleDeg(0.0f - gyroYawDeg);
  const float wz = constrain(
      HIGH_GYRO_KP * yawErrorDeg * DEG_TO_RAD,
      -HIGH_MAX_WZ, HIGH_MAX_WZ);
  setLocalVelocity(HIGH_STEP_CONTACT_SPEED_MPS, 0.0f, wz);
  sendTargetPacket();
}

static void driveHigherNoBoxForward()
{
  const float yawErrorDeg = normalizeAngleDeg(0.0f - gyroYawDeg);
  const float wz = constrain(
      HIGH_GYRO_KP * yawErrorDeg * DEG_TO_RAD,
      -HIGH_MAX_WZ, HIGH_MAX_WZ);
  setLocalVelocity(HIGH_NO_BOX_FORWARD_SPEED_MPS, 0.0f, wz);
  sendTargetPacket();
}

void stopHigherStepAlignment(const char *reason)
{
  higherAlignActive = false;
  higherAlignState = HIGH_ALIGN_IDLE;
  higherAlignBothSinceMs = 0;
  higherAlignBackoffStartMs = 0;
  higherSoftLiftActive = false;
  boxParkingActive = false;
  if (hub.online)
    Serial1.println("LIDAR OFF");
  stopRobot();
  Serial.print("HIGH ALIGN STOP: ");
  Serial.println(reason);
}

bool startHigherStepAlignment(int32_t afterLiftFrontPulse,
                              int32_t afterLiftBackPulse)
{
  if (missionRunning)
  {
    Serial.println("HIGH ALIGN REJECTED: MISSION IS RUNNING");
    return false;
  }
  if (!hub.online)
  {
    Serial.println("HIGH ALIGN REJECTED: HUB OFFLINE");
    return false;
  }
  if (!gyroOnline)
  {
    Serial.println("HIGH ALIGN REJECTED: GYRO OFFLINE");
    return false;
  }
  if (liftRunning())
  {
    Serial.println("HIGH ALIGN REJECTED: LIFT IS RUNNING");
    return false;
  }
  if (hubInputActive(HUB_L_SW_FRONT) ||
      hubInputActive(HUB_R_SW_FRONT))
  {
    Serial.println("HIGH ALIGN REJECTED: FRONT LIMIT ALREADY ACTIVE");
    return false;
  }

  stopRobot();
  positionControlActive = false;
  boxParkingActive = false;
  velocityMode = VELOCITY_STOPPED;
  higherAlignTargetYawDeg = gyroYawDeg;
  higherAlignStartMs = millis();
  higherAlignBothSinceMs = 0;
  higherAlignBackoffStartMs = 0;
  higherAfterLiftFrontPulse = afterLiftFrontPulse;
  higherAfterLiftBackPulse = afterLiftBackPulse;
  higherAlignState = HIGH_ALIGN_START_LIFT;
  higherAlignActive = true;

  Serial.print("HIGH START LIFT: LP,");
  Serial.print(HIGH_START_LIFT_FRONT_PULSE);
  Serial.print(',');
  Serial.println(HIGH_START_LIFT_BACK_PULSE);
  startLift(HIGH_START_LIFT_FRONT_PULSE,
            HIGH_START_LIFT_BACK_PULSE,
            HIGH_LIFT_TIMEOUT_MS);
  return true;
}

void updateHigherStepAlignment()
{
  if (!higherAlignActive)
    return;

  const uint32_t now = millis();
  if (!hub.online)
  {
    stopHigherStepAlignment("HUB OFFLINE");
    return;
  }
  if (!gyroOnline)
  {
    stopHigherStepAlignment("GYRO OFFLINE");
    return;
  }
  if (higherAlignState != HIGH_ALIGN_START_LIFT &&
      higherAlignState != HIGH_ALIGN_AFTER_LIFT &&
      higherAlignState != HIGH_ALIGN_LIDAR &&
      higherAlignState != HIGH_ALIGN_NO_BOX_CONTACT_DRIVE &&
      higherAlignState != HIGH_ALIGN_NO_BOX_FRONT_ZERO &&
      higherAlignState != HIGH_ALIGN_NO_BOX_FORWARD_TF2 &&
      higherAlignState != HIGH_ALIGN_NO_BOX_BACK_ZERO &&
      higherAlignState != HIGH_ALIGN_NO_BOX_RETURN_START_HEIGHT &&
      higherAlignState != HIGH_ALIGN_NO_BOX_FORWARD_TF1 &&
      (uint32_t)(now - higherAlignStartMs) > HIGH_TIMEOUT_MS)
  {
    stopHigherStepAlignment("TIMEOUT");
    return;
  }

  if (higherAlignState == HIGH_ALIGN_START_LIFT)
  {
    stopRobot();

    if (liftDone())
    {
      higherAlignTargetYawDeg = gyroYawDeg;
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_APPROACH;
      Serial.print("HIGH START LIFT DONE: APPROACH YAW=");
      Serial.println(higherAlignTargetYawDeg, 2);
      return;
    }

    if (liftFailed())
    {
      stopHigherStepAlignment("START LIFT FAILED");
      return;
    }

    if ((uint32_t)(now - liftTaskStartMs) > HIGH_LIFT_TIMEOUT_MS)
    {
      liftTaskStatus = TASK_TIMEOUT;
      stopHigherStepAlignment("START LIFT TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_AFTER_LIFT)
  {
    stopRobot();
    updateHigherSoftLift();

    if (!higherSoftLiftActive && liftDone())
    {
      Serial.print("HIGH AFTER LIFT DONE: LP,");
      Serial.print(higherAfterLiftFrontPulse);
      Serial.print(',');
      Serial.print(higherAfterLiftBackPulse);
      Serial.println(" - START LIDAR BOX SEARCH");

      startBoxParking(HIGH_LIDAR_TARGET_DISTANCE_MM,
                      HIGH_LIDAR_TIMEOUT_MS,
                      false);
      higherAlignState = HIGH_ALIGN_LIDAR;
      return;
    }

    if (liftFailed())
    {
      stopHigherStepAlignment("LIFT FAILED");
      return;
    }

    if ((uint32_t)(now - liftTaskStartMs) > HIGH_LIFT_TIMEOUT_MS)
    {
      liftTaskStatus = TASK_TIMEOUT;
      stopHigherStepAlignment("LIFT TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_LIDAR)
  {
    updateBoxParking();

    // เมื่อพบกล่อง ให้ฟังก์ชันกล่องเดิมจัดระยะ กึ่งกลาง และมุมจนเสร็จ
    // updateBoxParking() จะหยุดฐานหุ่นและตั้ง TASK_DONE ให้อัตโนมัติ
    if (boxParkingDone())
    {
      higherAlignActive = false;
      higherAlignState = HIGH_ALIGN_IDLE;
      Serial1.println("LIDAR OFF");
      stopRobot();
      Serial.print("HIGH LIDAR RESULT: BOX FUNCTION DONE distance=");
      Serial.print(boxResult.distanceMm, 0);
      Serial.print(" offset=");
      Serial.print(boxResult.offsetMm, 0);
      Serial.print(" angle=");
      Serial.println(boxResult.angleDeg, 2);
      Serial.println("HIGH FUNCTION DONE: BOX ALIGNED AND STOPPED");
      return;
    }

    if (boxParkingFailed())
    {
      boxParkingActive = false;
      Serial1.println("LIDAR OFF");
      Serial.println("HIGH LIDAR RESULT: BOX NOT FOUND");
      Serial.println(
          "HIGH NO BOX: FORWARD BY TIME BEFORE FRONT LIFT");
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_NO_BOX_CONTACT_DRIVE;
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_NO_BOX_CONTACT_DRIVE)
  {
    // There is no contact sensor at this step. Drive forward for a tuned
    // amount of time so the front wheels touch the edge before moving the leg.
    driveHigherStepContact();

    if ((uint32_t)(now - higherAlignStartMs) >=
        HIGH_STEP_CONTACT_DRIVE_MS)
    {
      // The front wheels have reached the step. Stop before moving the
      // front lift, then wait for LP,0,<back> to finish.
      stopRobot();
      startLift(0,
                higherAfterLiftBackPulse,
                HIGH_LIFT_TIMEOUT_MS);
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_NO_BOX_FRONT_ZERO;

      Serial.print("HIGH NO BOX: CONTACT TIME DONE - START LP,0,");
      Serial.println(higherAfterLiftBackPulse);
      Serial.println(
          "HIGH NO BOX: WHEELS STOPPED - WAIT FRONT LIFT");
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_NO_BOX_FRONT_ZERO)
  {
    // Do not drive while the front lift is moving.
    stopRobot();

    if (liftDone())
    {
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_NO_BOX_FORWARD_TF2;
      Serial.println(
          "HIGH NO BOX: FRONT LIFT DONE - FORWARD UNTIL TF2");
      return;
    }

    if (liftFailed())
    {
      stopHigherStepAlignment("NO BOX FRONT LIFT FAILED");
      return;
    }

    if ((uint32_t)(now - liftTaskStartMs) >
        HIGH_LIFT_TIMEOUT_MS)
    {
      liftTaskStatus = TASK_TIMEOUT;
      stopHigherStepAlignment("NO BOX FRONT LIFT TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_NO_BOX_FORWARD_TF2)
  {
    driveHigherNoBoxForward();

    if (tf2ForwardStopDetected())
    {
      Serial.print("HIGH NO BOX: TF2 DETECTED distance=");
      Serial.print(hub.tfDistanceCm[1]);
      Serial.println(" cm");

      // Stop the wheels before moving the back lift.
      stopRobot();
      startLift(0, 0, HIGH_LIFT_TIMEOUT_MS);
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_NO_BOX_BACK_ZERO;
      Serial.println(
          "HIGH NO BOX: START LP,0,0 - WHEELS STOPPED");
      return;
    }

    if ((uint32_t)(now - higherAlignStartMs) >=
        HIGH_NO_BOX_FORWARD_TIMEOUT_MS)
    {
      stopHigherStepAlignment("NO BOX TF2 TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_NO_BOX_BACK_ZERO)
  {
    // Do not drive while the back lift is moving.
    stopRobot();

    if (liftDone())
    {
      // LP,0,0 is only the lowest transition point. Raise both lifts back to
      // the safe starting height before driving toward TF1.
      startLift(HIGH_START_LIFT_FRONT_PULSE,
                HIGH_START_LIFT_BACK_PULSE,
                HIGH_LIFT_TIMEOUT_MS);
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_NO_BOX_RETURN_START_HEIGHT;
      Serial.println(
          "HIGH NO BOX: LP,0,0 DONE - START LP,100,200");
      return;
    }

    if (liftFailed())
    {
      stopHigherStepAlignment("NO BOX BACK LIFT FAILED");
      return;
    }

    if ((uint32_t)(now - liftTaskStartMs) >
        HIGH_LIFT_TIMEOUT_MS)
    {
      liftTaskStatus = TASK_TIMEOUT;
      stopHigherStepAlignment("NO BOX BACK LIFT TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_NO_BOX_RETURN_START_HEIGHT)
  {
    // Keep the wheels stopped while returning from LP,0,0 to LP,100,200.
    stopRobot();

    if (liftDone())
    {
      higherAlignStartMs = now;
      higherAlignState = HIGH_ALIGN_NO_BOX_FORWARD_TF1;
      Serial.println(
          "HIGH NO BOX: LP,100,200 DONE - FORWARD UNTIL TF1");
      return;
    }

    if (liftFailed())
    {
      stopHigherStepAlignment("NO BOX RETURN START HEIGHT FAILED");
      return;
    }

    if ((uint32_t)(now - liftTaskStartMs) >
        HIGH_LIFT_TIMEOUT_MS)
    {
      liftTaskStatus = TASK_TIMEOUT;
      stopHigherStepAlignment("NO BOX RETURN START HEIGHT TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_NO_BOX_FORWARD_TF1)
  {
    driveHigherNoBoxForward();

    if (tf1ForwardStopDetected())
    {
      stopRobot();
      higherAlignActive = false;
      higherAlignState = HIGH_ALIGN_IDLE;
      Serial.print("HIGH NO BOX: TF1 DETECTED distance=");
      Serial.print(hub.tfDistanceCm[0]);
      Serial.println(" cm");
      Serial.println("HIGH FUNCTION DONE: STOP AT TF1");
      return;
    }

    if ((uint32_t)(now - higherAlignStartMs) >=
        HIGH_NO_BOX_FORWARD_TIMEOUT_MS)
    {
      stopHigherStepAlignment("NO BOX TF1 TIMEOUT");
      return;
    }
    return;
  }

  if (higherAlignState == HIGH_ALIGN_BACKOFF)
  {
    if ((uint32_t)(now - higherAlignBackoffStartMs) >= HIGH_BACKOFF_MS)
    {
      stopRobot();
      higherAlignBackoffStartMs = 0;
      Serial.print("HIGH BACKOFF DONE: START LP,");
      Serial.print(higherAfterLiftFrontPulse);
      Serial.print(',');
      Serial.print(higherAfterLiftBackPulse);
      Serial.println(" WITH SOFT START");
      startHigherSoftLift(higherAfterLiftFrontPulse,
                          higherAfterLiftBackPulse);
      higherAlignState = HIGH_ALIGN_AFTER_LIFT;
      return;
    }

    // Gyro zero was established from the field edge immediately before this
    // state. Hold that new 0-degree reference while reversing.
    const float yawErrorDeg = normalizeAngleDeg(0.0f - gyroYawDeg);
    const float wz = constrain(
        HIGH_GYRO_KP * yawErrorDeg * DEG_TO_RAD,
        -HIGH_MAX_WZ, HIGH_MAX_WZ);
    setLocalVelocity(-HIGH_BACKOFF_SPEED_MPS, 0.0f, wz);
    sendTargetPacket();
    return;
  }

  const bool leftPressed = hubInputActive(HUB_L_SW_FRONT);
  const bool rightPressed = hubInputActive(HUB_R_SW_FRONT);

  if (leftPressed && rightPressed)
  {
    setTargets(0.0f, 0.0f, 0.0f, 0.0f);
    sendTargetPacket();

    if (higherAlignState != HIGH_ALIGN_CONFIRM_BOTH)
    {
      higherAlignState = HIGH_ALIGN_CONFIRM_BOTH;
      higherAlignBothSinceMs = now;
      Serial.println("HIGH ALIGN: BOTH LIMITS - CONFIRMING");
    }
    else if ((uint32_t)(now - higherAlignBothSinceMs) >= HIGH_CONFIRM_MS)
    {
      stopRobot();
      resetGyroYaw();  // The field edge is now the true 0-degree reference.
      Serial.println("HIGH ALIGN DONE: FIELD ALIGNED, GYRO ZERO");
      higherAlignBackoffStartMs = now;
      higherAlignState = HIGH_ALIGN_BACKOFF;
      Serial.println("HIGH BACKOFF START: GYRO HOLD 0 DEG");
    }
    return;
  }

  // If either switch releases during confirmation, alignment must continue.
  higherAlignState = HIGH_ALIGN_APPROACH;
  higherAlignBothSinceMs = 0;

  constexpr float MPS_TO_RPM =
      60.0f / (2.0f * PI * WHEEL_RADIUS_M);
  const float crawlRpm = HIGH_ONE_SIDE_SPEED_MPS * MPS_TO_RPM;

  if (leftPressed)
  {
    // Left side stays still; only the right side crawls forward.
    setTargets(0.0f, crawlRpm, 0.0f, crawlRpm);
  }
  else if (rightPressed)
  {
    // Right side stays still; only the left side crawls forward.
    setTargets(crawlRpm, 0.0f, crawlRpm, 0.0f);
  }
  else
  {
    // Before touching the field, Gyro only keeps the approach heading.
    const float yawErrorDeg =
        normalizeAngleDeg(higherAlignTargetYawDeg - gyroYawDeg);
    const float wz = constrain(
        HIGH_GYRO_KP * yawErrorDeg * DEG_TO_RAD,
        -HIGH_MAX_WZ, HIGH_MAX_WZ);
    setLocalVelocity(HIGH_APPROACH_SPEED_MPS, 0.0f, wz);
  }

  sendTargetPacket();
}
