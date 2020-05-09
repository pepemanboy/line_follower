#ifndef LINE_FOLLOWER_CONTROL_H
#define LINE_FOLLOWER_CONTROL_H

#include <stdint.h>

#include "line_sensor.h"
#include "current_sensor.h"

namespace line_follower {

class Control {
public:
  Control();
  void Init();
  void Poll(uint32_t micros);

private:
  LineSensor line_sensor_ = {};
  CurrentSensor current_sensors_[kNumCurrentSensors] = {};
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_CONTROL_H
