#ifndef LINE_FOLLOWER_CURRENT_SENSOR_H
#define LINE_FOLLOWER_CURRENT_SENSOR_H

#include <stdint.h>

#include "low_pass_filter.h"

namespace line_follower {

class CurrentSensor {
public:
  CurrentSensor();  
  void Reset();

  void OnAnalogSample(int32_t analog_sample);
  float Output_Amps() const { return filter_A_.output(); }

private:
  LowPassFilter filter_A_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_CURRENT_SENSOR_H
