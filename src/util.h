#ifndef LINE_FOLLOWER_UTIL_H
#define LINE_FOLLOWER_UTIL_H

namespace line_follower {

template <typename T>
struct MaybeValid {
  T value;
  bool valid;
};

template <typename T>
struct MakeMaybeValid (T value, bool valid) {
  MaybeValid<T> m;
  m.value = value;
  m.valid = valid;
  return m;
}

}  // namespace line_follower

#endif  // LINE_FOLLOWER_UTIL_H