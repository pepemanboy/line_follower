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
  void ButtonsTest();
  void LineFollowerTest();
  void AdjustPidKp();
  void AdjustPidKd();
  void RangeSensorTest();
  void MotorsTest();
  void BatteryMeterTest();

  float ReadBluetoothFloat();
  int ReadBluetoothInt();

  float pid_kp_ = -1;
  float pid_kd_ = -1;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_FOLLOWER_STARTUP_TEST_H
