#include "current_sensor.h"

#include "constants.h"

constexpr float kMaxCurrent_A = 30.0f;
constexpr float kFilterWeight = 0.95f;

namespace line_follower {

CurrentSensor::CurrentSensor() : filter_A_(0.0f, kFilterWeight) {
  Reset();
}

void CurrentSensor::Reset() {
  filter_A_.Reset(0.0f);
}

void CurrentSensor::OnAnalogSample(int32_t analog_sample) {
  constexpr int32_t kAdcOffset = kAdcResolution / 2.0f;
  const float sample_A = (analog_sample - kAdcOffset) / kMaxCurrent_A;
  filter_A_.Update(sample_A);
}

}  // namespace line_follower