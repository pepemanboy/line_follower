#ifndef LINE_FOLLOWER_BATTERY_METER_H
#define LINE_FOLLOWER_BATTERY_METER_H

#include <stdint.h>

#include "low_pass_filter.h"

namespace line_follower {

class BatteryMeter {
public:
  BatteryMeter();
  void Reset();

  void OnAnalogSample(int32_t analog_sample);
  float Output_V() const { return filter_V_.output(); }

private:
  LowPassFilter filter_V_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BUTTON_DEBOUNCE_H