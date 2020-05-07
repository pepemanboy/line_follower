#ifndef LINE_FOLLOWER_LINE_SENSOR_H
#define LINE_FOLLOWER_LINE_SENSOR_H

#include <stdint.h>

#include "low_pass_filter.h"
#include "constants.h"
#include "util.h"

namespace line_follower {

class LineSensor {

public:
  LineSensor();
  void Reset();

  void OnQtrArrayReading(int32_t qtr_readings[kNumQtrSensors]);
  MaybeValid<float> Output_cm();

private:
  LowPassFilter filters_[kNumQtrSensors];
};


}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_SENSOR_H
