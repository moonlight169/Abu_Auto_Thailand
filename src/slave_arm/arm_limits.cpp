#include "arm_limits.h"

#include "motor.h"

DebouncedLimit topFrontLimit{TOP_LIMIT_FRONT, false, false, 0};
DebouncedLimit topBackLimit{TOP_LIMIT_BACK, false, false, 0};
DebouncedLimit bottomFrontLimit{BOTTOM_LIMIT_FRONT, false, false, 0};
DebouncedLimit bottomBackLimit{BOTTOM_LIMIT_BACK, false, false, 0};

static void updateOneLimit(DebouncedLimit &limit, uint32_t now) {
  const bool raw = isPressed(limit.pin);
  if (raw != limit.rawPressed) {
    limit.rawPressed = raw;
    limit.changedMs = now;
  }
  if (now - limit.changedMs >= LIMIT_DEBOUNCE_MS) {
    limit.stablePressed = limit.rawPressed;
  }
}

void updateDebouncedLimits() {
  const uint32_t now = millis();
  updateOneLimit(topFrontLimit, now);
  updateOneLimit(topBackLimit, now);
  updateOneLimit(bottomFrontLimit, now);
  updateOneLimit(bottomBackLimit, now);
}
