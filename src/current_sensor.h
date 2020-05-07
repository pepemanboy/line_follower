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
  float Output_Amps() { return filter_Amps.output(); }

private:
  LowPassFilter filter_Amps;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_CURRENT_SENSOR_H
