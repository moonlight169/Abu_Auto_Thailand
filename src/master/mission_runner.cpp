#include "mission_runner.h"

#include <math.h>

#include "arm_link.h"
#include "box_parking.h"
#include "box_result.h"
#include "gyro.h"
#include "hub_link.h"
#include "kinematics.h"
#include "lift_link.h"
#include "mission_list.h"
#include "odometry.h"
#include "position_control.h"
#include "robot_state.h"
#include "tasks.h"
#include "wheel_link.h"

bool missionRunning = false;
bool missionStepStarted = false;
size_t missionIndex = 0;

static uint32_t missionWaitStartMs = 0;

// ---- STEP_FORWARD_TF34_EDGE_FRONT_LIFT working state ----
static bool tf34EdgeFloorArmed = false;
static bool tf34EdgeLiftStarted = false;
static uint8_t tf3EdgeNoFloorCount = 0;
static uint8_t tf4EdgeNoFloorCount = 0;
static bool tf3EdgeSideStopped = false;
static bool tf4EdgeSideStopped = false;

// ---- STEP_FORWARD_TF1_BACK_LIFT working state ----
static bool tf1BackLiftStarted = false;
static bool tf1DescendWasTrue = false;
static uint8_t tf1DetectCount = 0;
static bool tf1DescendClearanceStarted = false;
static uint32_t tf1DescendClearanceStartMs = 0;

// ---- STEP_FORWARD_LZ_BACK working state ----
static ForwardLzBackState forwardLzBackState = FORWARD_LZ_BACK_IDLE;
static uint32_t forwardLzBackPhaseStartMs = 0;

// ---- STEP_LASER5_PICK_VERIFY working state ----
static Laser5PickState laser5PickState = LASER5_PICK_IDLE;
static uint32_t laser5PickActionMs = 0;

static void startCurrentMissionStep(const MissionStep &step);
static bool currentMissionStepDone(const MissionStep &step);
static bool currentMissionStepFailed(const MissionStep &step);

void stopMission(const char *reason)
{
  missionRunning = false;
  missionStepStarted = false;
  positionControlActive = false;
  boxParkingActive = false;
  liftMoveActive = false;
  liftHomeActive = false;

  if (armTaskStatus == ARM_TASK_RUNNING)
  {
    sendArmAuxCommand("STOP");
    armTaskStatus = ARM_TASK_IDLE;
    armTxPacket[0] = '\0';
  }

  if (moveTaskStatus == TASK_RUNNING)
    moveTaskStatus = TASK_ERROR;
  if (liftTaskStatus == TASK_RUNNING)
    liftTaskStatus = TASK_ERROR;
  if (boxParkingTaskStatus == TASK_RUNNING)
    boxParkingTaskStatus = TASK_ERROR;

  stopRobot();
  Serial.print("MISSION STOPPED: ");
  Serial.println(reason);
}

bool selectMission(uint8_t missionNumber)
{
  if (missionNumber < 1 || missionNumber > MISSION_PROGRAM_COUNT)
  {
    Serial.print("INVALID MISSION: ");
    Serial.println(missionNumber);
    return false;
  }

  if (missionRunning)
  {
    Serial.println("CANNOT CHANGE MISSION WHILE RUNNING");
    return false;
  }

  selectedMission = missionNumber - 1;
  Serial.print("MISSION SELECTED: ");
  Serial.print(missionNumber);
  Serial.print(" - ");
  Serial.println(activeMissionProgram().name);
  return true;
}

void startMission(uint8_t missionNumber)
{
  if (missionRunning)
  {
    Serial.println("MISSION ALREADY RUNNING");
    return;
  }

  if (missionNumber > 0 && !selectMission(missionNumber))
    return;

  missionIndex = 0;
  missionStepStarted = false;
  missionRunning = true;
  Serial.print("MISSION START: ");
  Serial.print(selectedMission + 1);
  Serial.print(" - ");
  Serial.println(activeMissionProgram().name);
}

bool startMissionByCode(const char *code)
{
  const int index = findMissionByName(code);

  if (index < 0)
  {
    Serial.print("UNKNOWN MISSION CODE: [");
    Serial.print(code != nullptr ? code : "");
    Serial.println("]");
    return false;
  }

  // startMission() numbers missions from 1 and takes a uint8_t, so entry 255
  // onwards cannot be addressed without widening that type.
  if (index >= 255)
  {
    Serial.print("MISSION INDEX OUT OF RANGE: ");
    Serial.println(index);
    return false;
  }

  startMission((uint8_t)(index + 1));
  return true;
}

void resetBeforeMission()
{
  stopRobot();

  // รีเซ็ต Odometry -- resetOdometry() is the only place that also clears
  // encoderHeadingRad, the heading the robot falls back to whenever the gyro
  // is offline. Zeroing X/Y alone left the world frame rotated by whatever
  // heading was still on the wheel encoders from the previous run, and every
  // MOVE_STEP after the reset assumes yaw = 0.
  resetOdometry();

  targetPositionX = 0.0f;
  targetPositionY = 0.0f;
  targetPositionYawRad = 0.0f;

  previousErrorX = 0.0f;
  previousErrorY = 0.0f;
  previousErrorYaw = 0.0f;

  positionDoneCounter = 0;
  positionControlActive = false;
  boxParkingActive = false;
  boxParkingTaskStatus = TASK_IDLE;
  boxParkingInToleranceSinceMs = 0;
  commandActive = false;

  // Home through startLiftHome() rather than a raw LZ so the lift task knows a
  // homing job is in flight. That is what lets parseLiftResponse() tell this
  // job's LIFT_HOME_REACHED apart from the LIFT_STEP that follows a moment
  // later, instead of letting it close that LIFT_STEP before the lift moves.
  startLiftHome(RESET_HOME_TIMEOUT_MS);

  resetGyroYaw();

  Serial.println("RESET STEP COMPLETE");
}

void resetBeforeMission2()
{
  stopRobot();

  // รีเซ็ต Odometry -- resetOdometry() is the only place that also clears
  // encoderHeadingRad, the heading the robot falls back to whenever the gyro
  // is offline. Zeroing X/Y alone left the world frame rotated by whatever
  // heading was still on the wheel encoders from the previous run, and every
  // MOVE_STEP after the reset assumes yaw = 0.
  resetOdometry();

  targetPositionX = 0.0f;
  targetPositionY = 0.0f;
  targetPositionYawRad = 0.0f;

  previousErrorX = 0.0f;
  previousErrorY = 0.0f;
  previousErrorYaw = 0.0f;

  positionDoneCounter = 0;
  positionControlActive = false;
  boxParkingActive = false;
  boxParkingTaskStatus = TASK_IDLE;
  boxParkingInToleranceSinceMs = 0;
  commandActive = false;

  resetGyroYaw();

  Serial.println("RESET 2 STEP COMPLETE");
}

static void startCurrentMissionStep(const MissionStep &step)
{
  Serial.print("MISSION STEP ");
  Serial.print(missionIndex);
  Serial.print(": ");

  switch (step.type)
  {

    case STEP_RESET:
    {
      Serial.println("RESET START");
      resetBeforeMission();
      Serial.println("RESET DONE - WAITING FOR LIFT HOME");
      break;
    }

    case STEP_RESET_2:
    {
      Serial.println("RESET START");
      resetBeforeMission2();
      Serial.println("RESET COMPLETE");
      break;
    }

    case STEP_LIFT:
      Serial.println("LIFT START");
      startLift((int32_t)lroundf(step.p1),
                (int32_t)lroundf(step.p2),
                step.timeoutMs);
      break;

    case STEP_MOVE:
      Serial.println("MOVE START");

      startMoveTo(
          step.p1,
          step.p2,
          step.p3,
          step.timeoutMs,
          step.maxSpeed);

      break;

    case STEP_LIDAR_PARK:
      Serial.println("LIDAR PARK START");
      startBoxParking(step.p1, step.timeoutMs);
      break;

    case STEP_WALL_ALIGN:
      Serial.println("WALL ALIGN START");
      startWallAlignment(step.p1, step.timeoutMs);
      break;

    case STEP_MOVE_UNTIL_FRONT:
      Serial.println("MOVE UNTIL L_SW_FRONT START");
      missionWaitStartMs = millis();
      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
      }
      else if (hubInputActive(HUB_L_SW_FRONT))
      {
        stopRobot();
        Serial.println("L_SW_FRONT ALREADY ACTIVE");
      }
      else
      {
        commandLocalVelocity(fabsf(step.p1), 0.0f, 0.0f);
        commandActive = true;
        Serial.print("LOCAL FORWARD SPEED=");
        Serial.print(fabsf(step.p1), 3);
        Serial.println(" m/s");
      }
      break;

    case STEP_FORWARD_UNTIL_LASER3:
      Serial.println("FORWARD UNTIL LASER3 START");
      missionWaitStartMs = millis();
      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
      }
      else if (tf2ForwardStopDetected())
      {
        stopRobot();
        Serial.print("TF2 ALREADY < ");
        Serial.print(TF2_FORWARD_STOP_CM);
        Serial.println(" cm");
      }
      else
      {
        commandLocalVelocity(fabsf(step.p1), 0.0f, 0.0f);
        commandActive = true;
        Serial.print("LOCAL FORWARD SPEED=");
        Serial.print(fabsf(step.p1), 3);
        Serial.println(" m/s");
      }
      break;

    case STEP_FORWARD_UNTIL_LASER4:
      Serial.println("FORWARD UNTIL LASER4 START");
      missionWaitStartMs = millis();
      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
      }
      else if (tf1ForwardStopDetected())
      {
        stopRobot();
        Serial.print("TF1 ALREADY < ");
        Serial.print(TF1_FORWARD_STOP_CM);
        Serial.println(" cm");
      }
      else
      {
        commandLocalVelocity(fabsf(step.p1), 0.0f, 0.0f);
        commandActive = true;
        Serial.print("LOCAL FORWARD SPEED=");
        Serial.print(fabsf(step.p1), 3);
        Serial.println(" m/s");
      }
      break;

    case STEP_FORWARD_TF34_EDGE_FRONT_LIFT:
    {
      Serial.println("TF3/TF4 EDGE -> FRONT LIFT START");
      missionWaitStartMs = millis();
      tf34EdgeFloorArmed = false;
      tf34EdgeLiftStarted = false;
      tf3EdgeNoFloorCount = 0;
      tf4EdgeNoFloorCount = 0;
      tf3EdgeSideStopped = false;
      tf4EdgeSideStopped = false;

      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
        break;
      }

      const bool tf3Floor =
          tfDistanceInRange(2, TF3_MIN_CM, TF3_MAX_CM);
      const bool tf4Floor =
          tfDistanceInRange(3, TF4_MIN_CM, TF4_MAX_CM);
      tf34EdgeFloorArmed = tf3Floor && tf4Floor;

      // This step controls left/right wheel pairs directly. Keep the normal
      // velocity updater stopped so it cannot overwrite the split command.
      velocityMode = VELOCITY_STOPPED;
      commandedVx = 0.0f;
      commandedVy = 0.0f;
      commandedWz = 0.0f;
      rampedVx = 0.0f;
      rampedVy = 0.0f;
      rampedWz = 0.0f;
      constexpr float MPS_TO_RPM =
          60.0f / (2.0f * PI * WHEEL_RADIUS_M);
      const float forwardRpm = fabsf(step.p1) * MPS_TO_RPM;
      setTargets(forwardRpm, forwardRpm, forwardRpm, forwardRpm);
      sendTargetPacket();
      commandActive = true;
      Serial.print("LOCAL FORWARD SPEED=");
      Serial.print(fabsf(step.p1), 3);
      Serial.print(" m/s, FLOOR ARMED=");
      Serial.println(tf34EdgeFloorArmed ? "YES" : "NO");
      break;
    }

    case STEP_FORWARD_TF1_BACK_LIFT:
      Serial.println("FORWARD UNTIL TF1 -> BACK LIFT DOWN START");
      missionWaitStartMs = millis();
      tf1BackLiftStarted = false;
      tf1DescendWasTrue = false;
      tf1DetectCount = 0;
      tf1DescendClearanceStarted = false;
      tf1DescendClearanceStartMs = 0;

      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
      }
      else
      {
        commandLocalVelocity(fabsf(step.p1), 0.0f, 0.0f);
        commandActive = true;
        Serial.print("LOCAL FORWARD SPEED=");
        Serial.print(fabsf(step.p1), 3);
        Serial.println(" m/s, WAIT TF1 TRUE -> FALSE");
      }
      break;

    case STEP_FORWARD_LZ_BACK:
      Serial.println("FORWARD BY TIME -> LZ -> BACK BY TIME START");
      missionWaitStartMs = millis();
      forwardLzBackPhaseStartMs = millis();
      forwardLzBackState = FORWARD_LZ_BACK_FORWARD;
      commandLocalVelocity(fabsf(step.p1), 0.0f, 0.0f);
      commandActive = true;
      Serial.print("FORWARD ");
      Serial.print(fabsf(step.p1), 3);
      Serial.print(" m/s FOR ");
      Serial.print((uint32_t)lroundf(step.p2));
      Serial.println(" ms");
      break;

    case STEP_WAIT:
      Serial.println("WAIT START");
      missionWaitStartMs = millis();
      break;

    case STEP_HUB_WAIT_INPUT:
      Serial.print("WAIT HUB INPUT bit=");
      Serial.println((int)step.p1);
      missionWaitStartMs = millis();
      break;

    case STEP_HUB_RELAY:
      setHubRelay((uint8_t)lroundf(step.p1), step.p2 > 0.5f);
      break;

    case STEP_HUB_ARM:
      setHubArm((uint8_t)lroundf(step.p1));
      break;

    case STEP_HUB_SPIN:
      setHubSpin((uint8_t)lroundf(step.p1));
      break;

    case STEP_ARM_HOME:
      Serial.println("ARM HOME START");
      startArmHome();
      break;

    case STEP_ARM_BOTTOM:
      Serial.println("ARM BOTTOM POSITION START");
      startArmBottom(step.p1);
      break;

    case STEP_ARM_TOP:
      Serial.println("ARM TOP POSITION START");
      startArmTop(step.p1);
      break;

    case STEP_ARM_POS:
      Serial.println("ARM TWO-AXIS POSITION START");
      startArmPosition(step.p1, step.p2);
      break;

    case STEP_SLIDE_UNTIL_LASER5:
    {
      Serial.println("SLIDE UNTIL LASER5 START");

      missionWaitStartMs = millis();

      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
      }
      else if (hubLaserDetected(HUB_LASER5))
      {
        stopRobot();
        Serial.println("LASER5 ALREADY ACTIVE");
      }
      else
      {
        const float slideVy   = step.p1;
        const float forwardVx = step.p2;

        commandLocalVelocity(
            forwardVx,
            slideVy,
            0.0f);

        commandActive = true;

        Serial.print("LOCAL VELOCITY vx=");
        Serial.print(forwardVx, 3);
        Serial.print(" vy=");
        Serial.print(slideVy, 3);
        Serial.println(" m/s");
      }

      break;
    }

    case STEP_LASER5_PICK_VERIFY:
    {
      Serial.println("LASER5 PICK AND VERIFY START");
      missionWaitStartMs = millis();
      laser5PickState = LASER5_PICK_SEARCHING;

      if (!hub.online)
      {
        stopRobot();
        Serial.println("ERROR: HUB OFFLINE");
      }
      else if (hubLaserDetected(HUB_LASER5))
      {
        stopRobot();
        setHubRelay(4, false);
        setHubArm(0);
        laser5PickActionMs = millis();
        laser5PickState = LASER5_PICK_WAIT_VERIFY;
        Serial.println("LASER5 ACTIVE: RELAY4=0, ARM=0");
      }
      else
      {
        commandLocalVelocity(step.p2, step.p1, 0.0f);
        commandActive = true;
        Serial.println("SLIDING TO SEARCH WORKPIECE");
      }
      break;
    }

    case STEP_END:
      break;
  }
}

static bool currentMissionStepDone(const MissionStep &step)
{
  switch (step.type)
  {
    case STEP_RESET:
    {
      // Wait for the homing that resetBeforeMission() started, so the lift
      // encoders are genuinely zeroed before the run. liftHomeReached is only
      // set by a reply that belongs to a homing job, so the LIFT_STEP that
      // follows cannot be what closes this wait.
      if (liftDone() && liftHomeReached)
      {
        Serial.println("RESET HOME COMPLETE");
        return true;
      }

      // The wait is capped by RESET_HOME_TIMEOUT_MS, armed on the lift task by
      // startLiftHome(). A lift that cannot home is a problem worth shouting
      // about, but it must not keep the robot in the start zone, so the step
      // gives up loudly and the mission continues.
      if (liftFailed())
      {
        Serial.print("WARNING: RESET HOME DID NOT COMPLETE - CONTINUING AT front=");
        Serial.print(liftPosFront);
        Serial.print(" back=");
        Serial.println(liftPosBack);
        return true;
      }

      return false;
    }

    case STEP_RESET_2:
      return true;

    case STEP_LIFT:
      return liftDone();

    case STEP_MOVE:
      return moveDone();

    case STEP_LIDAR_PARK:
      return boxParkingDone();

    case STEP_WALL_ALIGN:
      return boxParkingDone();

    case STEP_MOVE_UNTIL_FRONT:
      if (hubInputActive(HUB_L_SW_FRONT))
      {
        stopRobot();
        Serial.println("L_SW_FRONT PRESSED - ROBOT STOP");
        return true;
      }
      return false;

    case STEP_FORWARD_UNTIL_LASER3:
      if (tf2ForwardStopDetected())
      {
        stopRobot();
        commandActive = false;
        Serial.print("TF2=");
        Serial.print(hub.tfDistanceCm[1]);
        Serial.print(" cm < ");
        Serial.print(TF2_FORWARD_STOP_CM);
        Serial.println(" cm - ROBOT STOP");
        return true;
      }
      return false;

    case STEP_FORWARD_UNTIL_LASER4:
      if (tf1ForwardStopDetected())
      {
        stopRobot();
        commandActive = false;
        Serial.print("TF1=");
        Serial.print(hub.tfDistanceCm[0]);
        Serial.print(" cm < ");
        Serial.print(TF1_FORWARD_STOP_CM);
        Serial.println(" cm - ROBOT STOP");
        return true;
      }
      return false;

    case STEP_FORWARD_TF34_EDGE_FRONT_LIFT:
    {
      if (tf34EdgeLiftStarted)
        return liftDone();

      const bool tf34Valid =
          (hub.tfValidMask & ((1u << 2) | (1u << 3))) ==
          ((1u << 2) | (1u << 3));
      if (!tf34Valid)
      {
        // Do not release a side that has already stopped. Invalid readings
        // only reset confirmation on a side that is still moving.
        if (!tf3EdgeSideStopped)
          tf3EdgeNoFloorCount = 0;
        if (!tf4EdgeSideStopped)
          tf4EdgeNoFloorCount = 0;
        return false;
      }

      const bool tf3Floor =
          tfDistanceInRange(2, TF3_MIN_CM, TF3_MAX_CM);
      const bool tf4Floor =
          tfDistanceInRange(3, TF4_MIN_CM, TF4_MAX_CM);

      if (!tf34EdgeFloorArmed)
      {
        if (tf3Floor && tf4Floor)
        {
          tf34EdgeFloorArmed = true;
          tf3EdgeNoFloorCount = 0;
          tf4EdgeNoFloorCount = 0;
          Serial.println("TF3/TF4 FLOOR CONFIRMED - EDGE ARMED");
        }
        return false;
      }

      if (!tf3EdgeSideStopped)
      {
        if (!tf3Floor)
        {
          if (tf3EdgeNoFloorCount < TF34_EDGE_CONFIRM_COUNT)
            tf3EdgeNoFloorCount++;
        }
        else
        {
          tf3EdgeNoFloorCount = 0;
        }

        if (tf3EdgeNoFloorCount >= TF34_EDGE_CONFIRM_COUNT)
        {
          tf3EdgeSideStopped = true;
          Serial.print("TF3 EDGE CONFIRMED - ");
          Serial.println(TF3_EDGE_IS_LEFT_SIDE
              ? "LEFT WHEELS STOP"
              : "RIGHT WHEELS STOP");
        }
      }

      if (!tf4EdgeSideStopped)
      {
        if (!tf4Floor)
        {
          if (tf4EdgeNoFloorCount < TF34_EDGE_CONFIRM_COUNT)
            tf4EdgeNoFloorCount++;
        }
        else
        {
          tf4EdgeNoFloorCount = 0;
        }

        if (tf4EdgeNoFloorCount >= TF34_EDGE_CONFIRM_COUNT)
        {
          tf4EdgeSideStopped = true;
          Serial.print("TF4 EDGE CONFIRMED - ");
          Serial.println(TF3_EDGE_IS_LEFT_SIDE
              ? "RIGHT WHEELS STOP"
              : "LEFT WHEELS STOP");
        }
      }

      const bool leftStopped = TF3_EDGE_IS_LEFT_SIDE
          ? tf3EdgeSideStopped
          : tf4EdgeSideStopped;
      const bool rightStopped = TF3_EDGE_IS_LEFT_SIDE
          ? tf4EdgeSideStopped
          : tf3EdgeSideStopped;

      constexpr float MPS_TO_RPM =
          60.0f / (2.0f * PI * WHEEL_RADIUS_M);
      const float forwardRpm = fabsf(step.p1) * MPS_TO_RPM;
      setTargets(
          leftStopped ? 0.0f : forwardRpm,
          rightStopped ? 0.0f : forwardRpm,
          leftStopped ? 0.0f : forwardRpm,
          rightStopped ? 0.0f : forwardRpm);
      sendTargetPacket();

      if (!leftStopped || !rightStopped)
        return false;

      stopRobot();
      commandActive = false;
      tf34EdgeLiftStarted = true;

      const int32_t frontPulse = (int32_t)lroundf(step.p2);
      const int32_t backPulse = (int32_t)lroundf(step.p3);
      Serial.print("TF3/TF4 EDGES CONFIRMED - ALL WHEELS STOP, LP,");
      Serial.print(frontPulse);
      Serial.print(",");
      Serial.println(backPulse);
      startLift(frontPulse, backPulse, step.timeoutMs);
      return false;
    }

    case STEP_FORWARD_TF1_BACK_LIFT:
    {
      if (tf1BackLiftStarted)
        return liftDone();

      // TF1 has already confirmed the edge. Continue forward slowly for
      // the configured clearance time before stopping and lowering the
      // back lift. Sensor changes during this phase are intentionally
      // ignored because the edge transition has already been latched.
      if (tf1DescendClearanceStarted)
      {
        if ((uint32_t)(millis() - tf1DescendClearanceStartMs) <
            TF1_DESCEND_CLEARANCE_MS)
        {
          return false;
        }

        stopRobot();
        commandActive = false;
        tf1BackLiftStarted = true;

        const int32_t frontPulse = liftPosFront;
        const int32_t backPulse = (int32_t)lroundf(step.p2);
        Serial.print("TF1 EDGE CLEARANCE DONE - ALL WHEELS STOP, KEEP FRONT=");
        Serial.print(frontPulse);
        Serial.print(", BACK LIFT=");
        Serial.println(backPulse);
        startLift(frontPulse, backPulse, step.timeoutMs);
        return false;
      }

      constexpr uint8_t TF1_INDEX = 0;
      const bool tf1Valid =
          hub.online &&
          (hub.tfValidMask & (1u << TF1_INDEX)) != 0;

      if (!tf1Valid)
      {
        tf1DetectCount = 0;
        return false;
      }

      const uint16_t tf1DistanceCm = hub.tfDistanceCm[TF1_INDEX];
      const bool tf1Now = tf1DescendFloorDetected();

      // Descend detection must start from TRUE first. This prevents the robot
      // from stopping immediately if the step begins while TF1 sees no floor.
      if (!tf1DescendWasTrue)
      {
        if (tf1Now)
        {
          tf1DescendWasTrue = true;
          tf1DetectCount = 0;
          Serial.print("TF1 DESCEND ARMED: TRUE, distance=");
          Serial.print(tf1DistanceCm);
          Serial.println(" cm");
        }
        return false;
      }

      if (!tf1Now && tf1DistanceCm > TF1_DESCEND_FLOOR_MAX_CM)
      {
        if (tf1DetectCount < TF1_DETECT_CONFIRM_COUNT)
          ++tf1DetectCount;
      }
      else
      {
        tf1DetectCount = 0;
      }

      if (tf1DetectCount < TF1_DETECT_CONFIRM_COUNT)
        return false;

      tf1DescendClearanceStarted = true;
      tf1DescendClearanceStartMs = millis();
      commandLocalVelocity(TF1_DESCEND_CLEARANCE_SPEED_MPS, 0.0f, 0.0f);
      commandActive = true;

      Serial.print("TF1 TRUE -> FALSE CONFIRMED, distance=");
      Serial.print(tf1DistanceCm);
      Serial.print(" cm - CLEARANCE FOR ");
      Serial.print(TF1_DESCEND_CLEARANCE_MS);
      Serial.print(" ms AT ");
      Serial.print(TF1_DESCEND_CLEARANCE_SPEED_MPS, 3);
      Serial.println(" m/s");
      return false;
    }

    case STEP_FORWARD_LZ_BACK:
    {
      const uint32_t now = millis();

      if (forwardLzBackState == FORWARD_LZ_BACK_FORWARD)
      {
        const uint32_t forwardMs = (uint32_t)lroundf(step.p2);
        if ((uint32_t)(now - forwardLzBackPhaseStartMs) < forwardMs)
          return false;

        stopRobot();
        commandActive = false;
        startLiftHome(step.timeoutMs);
        forwardLzBackState = FORWARD_LZ_BACK_WAIT_HOME;
        Serial.println("FORWARD DONE - ROBOT STOP, LZ SENT");
        return false;
      }

      if (forwardLzBackState == FORWARD_LZ_BACK_WAIT_HOME)
      {
        // startLiftHome() resets liftHomeReached. Do not reverse until the
        // lift controller returns a new LIFT_HOME_REACHED response.
        if (!liftDone() || !liftHomeReached)
          return false;

        commandLocalVelocity(-fabsf(step.p3), 0.0f, 0.0f);
        commandActive = true;
        forwardLzBackPhaseStartMs = now;
        forwardLzBackState = FORWARD_LZ_BACK_REVERSE;
        Serial.print("LZ HOME DONE - BACKWARD ");
        Serial.print(fabsf(step.p3), 3);
        Serial.print(" m/s FOR ");
        Serial.print((uint32_t)lroundf(step.maxSpeed));
        Serial.println(" ms");
        return false;
      }

      if (forwardLzBackState == FORWARD_LZ_BACK_REVERSE)
      {
        const uint32_t backMs = (uint32_t)lroundf(step.maxSpeed);
        if ((uint32_t)(now - forwardLzBackPhaseStartMs) < backMs)
          return false;

        stopRobot();
        commandActive = false;
        forwardLzBackState = FORWARD_LZ_BACK_DONE;
        Serial.println("BACKWARD TIME DONE - ROBOT STOP");
        return true;
      }

      return forwardLzBackState == FORWARD_LZ_BACK_DONE;
    }

    case STEP_SLIDE_UNTIL_LASER5:
      if (hubLaserDetected(HUB_LASER5))
      {
        stopRobot();
        Serial.println("LASER5 DETECTED - ROBOT STOP");
        return true;
      }

      return false;

    case STEP_LASER5_PICK_VERIFY:
      switch (laser5PickState)
      {
        case LASER5_PICK_SEARCHING:
          if (hubLaserDetected(HUB_LASER5))
          {
            stopRobot();
            commandActive = false;
            setHubRelay(4, false);
            setHubArm(0);
            laser5PickActionMs = millis();
            laser5PickState = LASER5_PICK_WAIT_VERIFY;
            Serial.println("LASER5 DETECTED: RELAY4=0, ARM=0");
          }
          return false;

        case LASER5_PICK_WAIT_VERIFY:
          stopRobot();
          commandActive = false;

          if ((uint32_t)(millis() - laser5PickActionMs) <
              LASER5_ARM_VERIFY_WAIT_MS)
          {
            return false;
          }

          if (hubLaserDetected(HUB_LASER5))
          {
            laser5PickState = LASER5_PICK_DONE;
            Serial.println("LASER5 VERIFY OK - WORKPIECE HELD");
            return true;
          }

          // จับไม่สำเร็จ: ต้องอ้ากริปเปอร์ก่อนยืดแขน
          setHubRelay(4, true);
          laser5PickActionMs = millis();
          laser5PickState = LASER5_PICK_WAIT_GRIPPER_OPEN;
          Serial.println("LASER5 VERIFY FAILED: RELAY4=1, OPEN GRIPPER");
          return false;

        case LASER5_PICK_WAIT_GRIPPER_OPEN:
          stopRobot();
          commandActive = false;

          if ((uint32_t)(millis() - laser5PickActionMs) <
              LASER5_GRIPPER_OPEN_WAIT_MS)
          {
            return false;
          }

          // รอให้กริปเปอร์อ้าแล้วจึงยืดแขนกลับไปตำแหน่งพร้อมหยิบ
          // มุมมาจาก step.p3 = อาร์กิวเมนต์ที่ 3 ของ LASER5_PICK_VERIFY_STEP
          setHubArm((uint8_t)lroundf(step.p3));
          laser5PickActionMs = millis();
          laser5PickState = LASER5_PICK_WAIT_ARM_RETURN;
          Serial.print("GRIPPER OPENED: ARM=");
          Serial.println((int)lroundf(step.p3));
          return false;

        case LASER5_PICK_WAIT_ARM_RETURN:
          stopRobot();
          commandActive = false;

          if ((uint32_t)(millis() - laser5PickActionMs) <
              LASER5_ARM_RETURN_WAIT_MS)
          {
            return false;
          }

          // แขนถึงตำแหน่งรอหยิบแล้ว จึงเริ่มสไลด์ค้นหาใหม่
          commandLocalVelocity(step.p2, step.p1, 0.0f);
          commandActive = true;
          laser5PickState = LASER5_PICK_SEARCHING;
          Serial.println("RESUME SLIDING TO SEARCH WORKPIECE");
          return false;

        case LASER5_PICK_DONE:
          return true;

        default:
          return false;
      }

    case STEP_WAIT:
      return (uint32_t)(millis() - missionWaitStartMs) >=
             step.timeoutMs;

    case STEP_HUB_WAIT_INPUT:
      return hubInputActive((uint8_t)lroundf(step.p1)) ==
             (step.p2 > 0.5f);

    case STEP_HUB_RELAY:
    case STEP_HUB_ARM:
    case STEP_HUB_SPIN:
      return true;

    case STEP_ARM_HOME:
    case STEP_ARM_BOTTOM:
    case STEP_ARM_TOP:
    case STEP_ARM_POS:
      return armTaskStatus == ARM_TASK_DONE;

    case STEP_END:
      return true;
  }

  return false;
}

static bool currentMissionStepFailed(const MissionStep &step)
{
  switch (step.type)
  {
    case STEP_LIFT:
      return liftFailed();
    case STEP_MOVE:
      return moveFailed();
    case STEP_LIDAR_PARK:
      return boxParkingFailed();
    case STEP_WALL_ALIGN:
      return boxParkingFailed();
    case STEP_MOVE_UNTIL_FRONT:
      return !hub.online ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_FORWARD_UNTIL_LASER3:
      return !hub.online ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_FORWARD_UNTIL_LASER4:
      return !hub.online ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_FORWARD_TF34_EDGE_FRONT_LIFT:
      return (!tf34EdgeLiftStarted && !hub.online) ||
             (tf34EdgeLiftStarted && liftFailed()) ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_FORWARD_TF1_BACK_LIFT:
      return (!tf1BackLiftStarted && !hub.online) ||
             (tf1BackLiftStarted && liftFailed()) ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_FORWARD_LZ_BACK:
      return (forwardLzBackState == FORWARD_LZ_BACK_WAIT_HOME &&
              liftFailed()) ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_SLIDE_UNTIL_LASER5:
      return !hub.online ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_LASER5_PICK_VERIFY:
      return !hub.online ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_HUB_WAIT_INPUT:
      return !hub.online ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - missionWaitStartMs) >= step.timeoutMs);
    case STEP_ARM_HOME:
    case STEP_ARM_BOTTOM:
    case STEP_ARM_TOP:
    case STEP_ARM_POS:
      return armTaskStatus == ARM_TASK_ERROR ||
             (step.timeoutMs > 0 &&
              (uint32_t)(millis() - armCommandStartMs) >= step.timeoutMs);
    default:
      return false;
  }
}

void updateMission()
{
  if (!missionRunning)
    return;

  const MissionProgram &program = activeMissionProgram();

  if (missionIndex >= program.stepCount)
  {
    stopMission("INVALID STEP INDEX");
    return;
  }

  const MissionStep &step = program.steps[missionIndex];

  if (step.type == STEP_END)
  {
    missionRunning = false;
    missionStepStarted = false;
    stopRobot();
    Serial.print("MISSION FINISHED: ");
    Serial.println(selectedMission + 1);
    return;
  }

  if (!missionStepStarted)
  {
    startCurrentMissionStep(step);
    missionStepStarted = true;
  }

  if (currentMissionStepFailed(step))
  {
    Serial.print("MISSION ERROR AT STEP ");
    Serial.println(missionIndex);
    stopMission("STEP FAILED OR TIMEOUT");
    return;
  }

  if (currentMissionStepDone(step))
  {
    Serial.print("MISSION STEP COMPLETE: ");
    Serial.println(missionIndex);
    missionIndex++;
    missionStepStarted = false;
  }
}
