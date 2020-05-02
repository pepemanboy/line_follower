#include <Arduino.h>
#include "line_follower_startup_test.h"

namespace {
  line_follower::LineFollowerStartupTest robot;
}  // namespace

void setup() {
  robot.Init();
}

void loop() {
  robot.Poll(micros());
}
