#include "util.h"

#include <math.h>

namespace line_follower {

template <typename T>
MaybeValid<T> MakeMaybeValid (T value, bool valid) {
  MaybeValid<T> m;
  m.value = value;
  m.valid = valid;
  return m;
}

MaybeValid<Stats> WeightedStats(
    const float * weights, const float *observations, int n) {
  float weights_sum = 0;
  float weights_observations_sum = 0;

  // Weighted average.
  for (int i = 0; i < n; ++i) {
    weights_sum += weights[i];
    weights_observations_sum += weights[i] * observations[i];
  }
  if (weights_sum == 0.0f) return MakeMaybeValid<Stats>({}, false);
  
  const float weighted_average = weights_observations_sum / weights_sum;

  // Weighted stddev. 
  // See https://en.wikipedia.org/wiki/Reduced_chi-squared_statistic.
  float stddev_sum = 0;
  for (int i = 0; i < n; ++i) {   
    const float x = (observations[i] - weighted_average);
    stddev_sum += weights[i] * x * x;
  }
  const float weighted_stddev = sqrt(stddev_sum / weights_sum);

  return MakeMaybeValid<Stats>({weighted_average, weighted_stddev}, true);
}

}