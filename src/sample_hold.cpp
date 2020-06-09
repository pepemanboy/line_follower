#include "sample_hold.h"

#include <stdint.h>

namespace line_follower {

SampleHold::SampleHold(uint32_t hold_micros):
  hold_micros_(hold_micros) {}

void SampleHold::OnDigitalRead(uint32_t micros, bool reading) {
  if (reading) {
    output_ = true;
    last_high_micros_ = micros;
  } else if (micros - last_high_micros_ > hold_micros_) {
    output_ = false;
  } else {
    output_ = true;
  }
}

bool SampleHold::output() {
  return output_;
}

}  // namespace line_follower
