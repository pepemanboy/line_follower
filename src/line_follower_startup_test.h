#ifndef LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H
#define LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H

#include <stdint.h>

#include "line_sensor.h"
#include "current_sensor.h"

namespace line_follower {

class LineFollowerStartupTest {
public:
  LineFollowerStartupTest();

  void Init();

  void Poll(uint32_t micros);

private:
  void StartupTest();
  void SensorsTest();
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H
