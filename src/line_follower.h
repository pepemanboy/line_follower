#ifndef LINE_FOLLOWER_LINE_FOLLOWER_H
#define LINE_FOLLOWER_LINE_FOLLOWER_H

#include <stdint.h>

#include "hardware.h"
#include "control.h"

namespace line_follower {

class LineFollower {
public:
  LineFollower();

  void Init();

  void Poll(uint32_t micros);

private:
  void UpdateTower(uint32_t micros, Control::State state);
  Control control_ = {};
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_FOLLOWER_H
