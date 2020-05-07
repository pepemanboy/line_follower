#include "line_sensor.h"

namespace line_follower {


LineSensor::LineSensor() {
  Reset();
}

void LineSensor::Reset() {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    filters_[i].Reset(0.0f);
  }
}

void LineSensor::OnQtrArrayReading(int32_t qtr_readings[kNumQtrSensors]) {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    filters_[i].Update(qtr_readings[i]);
  }
}

MaybeValid<float> LineSensor::Output_cm() {

}


}  // namespace line_follower
