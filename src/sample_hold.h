#ifndef LINE_FOLLOWER_SAMPLE_HOLD_H
#define LINE_FOLLOWER_SAMPLE_HOLD_H

#include <stdint.h>

namespace line_follower {

class SampleHold {
public:
  explicit SampleHold(uint32_t hold_micros);
  void OnDigitalRead(uint32_t micros, bool reading);
  bool output();

private:
  bool output_ = false;
  uint32_t last_high_micros_ = 0;
  const uint32_t hold_micros_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_SAMPLE_HOLD_H
