#include "button_debounce.h"

#include <stdint.h>

constexpr uint32_t kDebounce_micros = 300000UL;

namespace line_follower {

ButtonDebounce::ButtonDebounce() {}

void ButtonDebounce::OnDigitalRead(uint32_t micros, bool reading) {
  if (micros - last_toggle_micros_ < kDebounce_micros)
    return;
  
  // Edge.
  if (output_ != reading) {
    last_toggle_micros_ = micros;
    // Rising edge.
    if (reading) {
      pulse_ = true;
    }
  }
    

  output_ = reading;

}

bool ButtonDebounce::Pulse() {
  if (pulse_) {
    pulse_ = false;
    return true;
  }
  return false;
}

}  // namespace line_follower