#ifndef LINE_FOLLOWER_LINE_SENSOR_H
#define LINE_FOLLOWER_LINE_SENSOR_H

#include <stdint.h>

#include "util.h"
#include "low_pass_filter.h"
#include "constants.h"

namespace line_follower {

class LineSensor {
public:
  LineSensor();
  void Reset();

  void OnQtrArrayReading(int32_t qtr_readings[kNumQtrSensors]);
  MaybeValid<Stats> MaybeOutput_mm();

private:
  float SensorPosition_mm(int sensor_index) const;
  LowPassFilter filters_[kNumQtrSensors];
  float sensor_positions_mm_[kNumQtrSensors];
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_SENSOR_H
