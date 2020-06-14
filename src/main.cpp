#include <Arduino.h>
#include "line_follower_startup_test.h"
#include "line_follower.h"

// Uncomment for test mode.
// #define TEST

namespace {
#ifdef TEST
  line_follower::LineFollowerStartupTest robot;
#else
  line_follower::LineFollower robot;
#endif
}  // namespace

void setup() {
  robot.Init();
}

void loop() {
  robot.Poll(micros());
}
