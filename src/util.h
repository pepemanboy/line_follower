#ifndef LINE_FOLLOWER_UTIL_H
#define LINE_FOLLOWER_UTIL_H

namespace line_follower {

template <typename T>
struct MaybeValid {
  T value;
  bool valid;
};

template <typename T>
MaybeValid<T> MakeMaybeValid(T value, bool valid);

struct Stats {
  float average;
  float std_dev;
};

MaybeValid<Stats> WeightedStats(
    const float * weights, const float *observations, int n);

}  // namespace line_follower

#endif  // LINE_FOLLOWER_UTIL_H