#ifndef LINE_FOLLOWER_LINE_FOLLOWER_H
#define LINE_FOLLOWER_LINE_FOLLOWER_H

#include <stdint.h>

#include "hardware.h"
#include "control.h"
#include "button_debounce.h"

namespace line_follower {

class LineFollower {
public:
  LineFollower();
  ~LineFollower();

  void Init();

  void Poll(uint32_t micros);

  void SetControlPidKp(float pid_kp) { control_.set_pid_kp(pid_kp); }
  float ControlPidKp() { return control_.pid_kp(); }

  void SetControlPidKd(float pid_kd) { control_.set_pid_kd(pid_kd); }
  float ControlPidKd() { return control_.pid_kd(); }

private:
  void UpdateTower(uint32_t micros, Control::State state);  
  Control control_ = {};
  ButtonDebounce button_up_ = {};
  ButtonDebounce button_down_ = {};
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_LINE_FOLLOWER_H
