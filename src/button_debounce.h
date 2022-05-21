#ifndef LINE_FOLLOWER_BUTTON_DEBOUNCE_H
#define LINE_FOLLOWER_BUTTON_DEBOUNCE_H

#include <stdint.h>

namespace line_follower {

class ButtonDebounce {
public:
  ButtonDebounce();
  void OnDigitalRead(uint32_t micros, bool reading);
  bool Pulse();

private:
  bool output_ = false;
  bool pulse_ = false;
  uint32_t last_toggle_micros_ = 0;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BUTTON_DEBOUNCE_H