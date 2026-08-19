#include "button_link.h"

#include <string.h>

#include "mission_runner.h"

static char buttonRxBuffer[BUTTON_RX_SIZE];
static size_t buttonRxIndex = 0;

// Set when a line overruns the buffer. Everything up to the next terminator
// is then thrown away. Resetting the index instead (the way readUsbSerial()
// does) would keep the TAIL of the long line, and a tail can be a valid code
// on its own -- the robot would drive off on garbage.
static bool buttonRxDiscard = false;

struct MissionCodeVote
{
  char code[BUTTON_RX_SIZE];
  uint8_t count;
};

static MissionCodeVote missionCodeVotes[BUTTON_VOTE_MAX_CANDIDATES];
static uint8_t missionCodeVoteCount = 0;
static uint8_t missionCodeFrameCount = 0;
static bool missionCodeVoteActive = false;
static bool missionCodeVoteOverflow = false;
static uint32_t missionCodeVoteStartMs = 0;

void beginButtonSerial()
{
  // A keypress is a few dozen bytes at 115200. The stock RX buffer is plenty,
  // so there is no addMemoryForRead() here the way the HUB stream needs.
  Serial3.begin(BUTTON_BAUD);
}

static void printRejectedCode(const char *source,
                              const char *reason,
                              const char *code)
{
  // Always echo the raw code. Which layer rejected it and what it actually
  // looked like are the two things needed to tell a wiring fault from a
  // mission that simply has not been written yet.
  Serial.print(source);
  Serial.print(" ");
  Serial.print(reason);
  Serial.print(": [");
  Serial.print(code);
  Serial.println("]");
}

// Returns the length a code starting with this Mode digit must have,
// or 0 when the digit is not a Mode the firmware knows.
static uint8_t expectedMissionCodeLength(char modeDigit)
{
  for (size_t i = 0; i < ARRAY_COUNT(MISSION_CODE_LENGTHS); i++)
  {
    if (MISSION_CODE_LENGTHS[i].modeDigit == modeDigit)
      return MISSION_CODE_LENGTHS[i].length;
  }

  return 0;
}

static void addMissionCodeVote(const char *code)
{
  const uint32_t now = millis();

  if (!missionCodeVoteActive)
  {
    missionCodeVoteActive = true;
    missionCodeVoteOverflow = false;
    missionCodeVoteStartMs = now;
    missionCodeVoteCount = 0;
    missionCodeFrameCount = 0;
  }

  if (missionCodeFrameCount < 255)
    missionCodeFrameCount++;

  for (uint8_t i = 0; i < missionCodeVoteCount; i++)
  {
    if (strcmp(missionCodeVotes[i].code, code) == 0)
    {
      if (missionCodeVotes[i].count < 255)
        missionCodeVotes[i].count++;
      return;
    }
  }

  if (missionCodeVoteCount >= BUTTON_VOTE_MAX_CANDIDATES)
  {
    missionCodeVoteOverflow = true;
    return;
  }

  strncpy(missionCodeVotes[missionCodeVoteCount].code,
          code,
          sizeof(missionCodeVotes[0].code) - 1);
  missionCodeVotes[missionCodeVoteCount].code[
      sizeof(missionCodeVotes[0].code) - 1] = '\0';
  missionCodeVotes[missionCodeVoteCount].count = 1;
  missionCodeVoteCount++;
}

static void updateMissionCodeVote()
{
  if (!missionCodeVoteActive)
    return;

  if ((uint32_t)(millis() - missionCodeVoteStartMs) < BUTTON_VOTE_WINDOW_MS)
    return;

  uint8_t bestIndex = 0;
  uint8_t bestCount = 0;
  bool tied = false;

  for (uint8_t i = 0; i < missionCodeVoteCount; i++)
  {
    if (missionCodeVotes[i].count > bestCount)
    {
      bestIndex = i;
      bestCount = missionCodeVotes[i].count;
      tied = false;
    }
    else if (missionCodeVotes[i].count == bestCount)
    {
      tied = true;
    }
  }

  // One code in the window wins outright however few frames carried it. That
  // is deliberate: requiring 2 frames would go completely silent if the keypad
  // ever drops back to sending once. With several codes competing, the winner
  // has to be alone at the top AND have at least two frames behind it, so two
  // separate corruptions must land on the exact same wrong code to get through.
  const bool accepted =
      !missionCodeVoteOverflow &&
      missionCodeVoteCount > 0 &&
      (missionCodeVoteCount == 1 || (!tied && bestCount >= 2));

  char winner[BUTTON_RX_SIZE];
  strncpy(winner, missionCodeVotes[bestIndex].code, sizeof(winner) - 1);
  winner[sizeof(winner) - 1] = '\0';

  const uint8_t frames = missionCodeFrameCount;
  const uint8_t candidates = missionCodeVoteCount;

  missionCodeVoteActive = false;
  missionCodeVoteOverflow = false;
  missionCodeVoteCount = 0;
  missionCodeFrameCount = 0;

  // The counters print through (int) on purpose: a bare uint8_t is a character
  // type, and not every core's Print treats it as a number.
  if (!accepted)
  {
    Serial.print("BUTTON VOTE FAILED - NO MAJORITY IN ");
    Serial.print((int)frames);
    Serial.print(" FRAMES, ");
    Serial.print((int)candidates);
    Serial.println(" DIFFERENT CODES - CHECK THE SERIAL3 WIRING");
    return;
  }

  Serial.print("BUTTON CODE VOTED: [");
  Serial.print(winner);
  Serial.print("] ");
  Serial.print((int)bestCount);
  Serial.print("/");
  Serial.print((int)frames);
  Serial.println(" FRAMES");

  startMissionByCode(winner);
}

void submitMissionCode(const char *code, const char *source)
{
  const size_t length = strlen(code);

  // layer 1: an empty line is normal line-ending noise, not a fault.
  if (length == 0)
    return;

  // layer 2: anything that is not a digit means the byte stream is damaged
  // or something else is talking on this port.
  for (size_t i = 0; i < length; i++)
  {
    if (code[i] < '0' || code[i] > '9')
    {
      printRejectedCode(source, "BAD CODE (NOT DIGITS)", code);
      return;
    }
  }

  // layer 3: the Mode digit fixes how long the code has to be. This is what
  // separates "the cable dropped characters" from "that mission does not
  // exist yet" (layer 4) -- two problems fixed in completely different places.
  // It only catches codes that are short or long; a flipped bit that keeps the
  // length is what the vote is for.
  const uint8_t expectedLength = expectedMissionCodeLength(code[0]);

  if (expectedLength == 0)
  {
    printRejectedCode(source, "BAD CODE (UNKNOWN MODE)", code);
    return;
  }

  if (length != expectedLength)
  {
    printRejectedCode(source, "BAD CODE (LENGTH)", code);
    return;
  }

  addMissionCodeVote(code);
}

void readButtonSerial()
{
  while (Serial3.available() > 0)
  {
    const char received = (char)Serial3.read();

    // '\0' counts as a terminator as well as '\n' and '\r'. The keypad
    // sends a real newline now that its TX buffer fits a 7-digit code plus
    // the '\n', so the NUL case is belt and braces. Keep it: a truncated
    // snprintf() on that board would silently put a NUL where the newline was.
    if (received == '\n' || received == '\r' || received == '\0')
    {
      if (buttonRxDiscard)
        buttonRxDiscard = false;
      else if (buttonRxIndex > 0)
      {
        buttonRxBuffer[buttonRxIndex] = '\0';
        submitMissionCode(buttonRxBuffer, "SERIAL3");
      }

      buttonRxIndex = 0;
      continue;
    }

    if (buttonRxDiscard)
      continue;

    // Everything outside printable ASCII is line noise and drops out here.
    if (received < 32 || received > 126)
      continue;

    if (buttonRxIndex >= BUTTON_RX_SIZE - 1)
    {
      buttonRxDiscard = true;
      buttonRxIndex = 0;
      Serial.println("SERIAL3 CODE TOO LONG - LINE DROPPED");
      continue;
    }

    buttonRxBuffer[buttonRxIndex++] = received;
  }

  updateMissionCodeVote();
}
