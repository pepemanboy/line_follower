#ifndef LINE_FOLLOWER_LOW_PASS_FILTER_H
#define LINE_FOLLOWER_LOW_PASS_FILTER_H

#include <stdint.h>

#include "line_sensor.h"

namespace line_follower {

class LowPassFilter {
public:
  explicit LowPassFilter(float initial_output, float weight);
  
  void Reset(float initial_output);
  void Update(float measurement); 

  void set_weight(float weight) { weight_ = weight; }
  float output() { return output_; }

private:
  float weight_;  // Init in constructor.
  float output_;   // Init in constructor.
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LOW_PASS_FILTER_H
