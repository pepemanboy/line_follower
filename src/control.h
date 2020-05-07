#ifndef LINE_FOLLOWER_CONTROL_H
#define LINE_FOLLOWER_CONTROL_H

#include <stdint.h>

#include "line_sensor.h"

class Control {
public:
  Control();
  void Init();
  void Poll(uint32_t micros);

private:
  LineSensor line_sensor_;
  CurrentSensor current_sensors_[kNumCurrentSensors];

};

#endif  // LINE_FOLLOWER_CONTROL_H
