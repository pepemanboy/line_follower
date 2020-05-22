#include "line_sensor.h"

#include "util.h"

constexpr float kQtrSensorSeparation_mm = 12.5f;
constexpr float kMaxQtrStdDev_mm = 40.0f;
constexpr float kAdcThreshold = 500;

namespace line_follower {

LineSensor::LineSensor() {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    sensor_positions_mm_[i] = SensorPosition_mm(i);
  }
  Reset();
}

void LineSensor::Reset() {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    filters_[i].Reset(0.0f);
  }
}

void LineSensor::OnQtrArrayReading(int32_t qtr_readings[kNumQtrSensors]) {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    float reading = qtr_readings[i] - kAdcThreshold;
    if (reading < 0) reading = 0;
    filters_[i].Update(reading);
  }
}

float LineSensor::SensorPosition_mm(int sensor_index) const {
  return kQtrSensorSeparation_mm * (sensor_index - kNumQtrSensors / 2 + 0.5f);
}

MaybeValid<Stats> LineSensor::MaybeOutput_mm() {
  float readings[kNumQtrSensors];
  for (int i = 0; i < kNumQtrSensors; ++i) {
    readings[i] = filters_[i].output();
  }
  MaybeValid<Stats> maybe_stats = 
    WeightedStats(readings, sensor_positions_mm_, kNumQtrSensors);

  if (maybe_stats.valid &&
      maybe_stats.value.std_dev > kMaxQtrStdDev_mm) {
    maybe_stats.valid = false;
  }

  return maybe_stats;
}

}  // namespace line_follower
