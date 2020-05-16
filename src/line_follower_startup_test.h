#ifndef LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H
#define LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H

#include <stdint.h>

namespace line_follower {

class LineFollowerStartupTest {
public:
  LineFollowerStartupTest();

  void Init();

  void Poll(uint32_t micros);

private:
  void StartupTest();
  void LineSensorsTest();
  void CurrentSensorsTest();
  void ButtonsTest();
  void LineFollowerTest();
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H
