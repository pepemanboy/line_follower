#include "battery_meter.h"

#include "constants.h"

namespace line_follower {

constexpr float kSampleToV = kAdcRef_V / kAdcResolution * ( 1.0 + 4.7);
constexpr float kFilterWeight = 0.3f;
constexpr float kDefault_V = 12.0f;

BatteryMeter::BatteryMeter(): filter_V_(kDefault_V, kFilterWeight) {
  Reset();
}

void BatteryMeter::Reset() {
  filter_V_.Reset(kDefault_V);
}

void BatteryMeter::OnAnalogSample(int32_t analog_sample) {
  const float sample_V = analog_sample * kSampleToV;
  filter_V_.Update(sample_V);
}

}  // namespace line_follower



