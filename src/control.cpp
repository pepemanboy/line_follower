#include "control.h"
#include "hardware.h"

namespace line_follower {

Control::Control() {

}

void Control::Init() {

}

void Control::Poll(uint32_t micros) {
  // Read QTR sensors.
  int32_t qtr_readings[kNumQtrSensors];
  ReadQtrSensors(qtr_readings);
  line_sensor_.OnQtrArrayReading(qtr_readings);

  // Read current sensors.
  int32_t current_readings[kNumCurrentSensors];
  ReadCurrentSensors(current_readings);
  for (int i = 0; i < kNumCurrentSensors; ++i) {
    current_sensors_[i].OnAnalogSample(current_readings[i]);
  }
}

}  // namespace line_follower