#include "current_sensor.h"

constexpr int kAdcResolution = 1024;
constexpr float kAdcOffset = kAdcResolution / 2.0f;
constexpr float kMaxCurrent_Amps = 30.0f;
constexpr float kFilterWeight = 0.95f;

namespace line_follower {

CurrentSensor::CurrentSensor() : filter_Amps(0.0f, kFilterWeight) {
  Reset();
}

void CurrentSensor::Reset() {
  filter_Amps.Reset(0.0f);
}

void CurrentSensor::OnAnalogSample(int32_t analog_sample) {
  const float sample_Amps = (analog_sample - kAdcOffset) / kMaxCurrent_Amps;
  filter_Amps.Update(sample_Amps);
}

}  // namespace line_follower