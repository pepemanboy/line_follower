#include "low_pass_filter.h"

namespace line_follower {

LowPassFilter::LowPassFilter(float initial_output, float weight) {
  set_weight(weight);
  Reset(initial_output);
}

void LowPassFilter::Reset(float initial_output) {
  output_ = initial_output;
}

void LowPassFilter::Update(float measurement) {
  output_ = weight_ * (measurement - output_) + output_;
}

}  // namespace line_follower