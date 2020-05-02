#include "line_follower.h"

#include <stdint.h>

#include <Arduino.h>

namespace line_follower {

LineFollower::LineFollower() {

}

void LineFollower::Init() {
  HardwareInit();
}

void LineFollower::Poll(uint32_t micros) {
}

}  // namespace line_follower
