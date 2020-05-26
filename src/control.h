#ifndef LINE_FOLLOWER_CONTROL_H
#define LINE_FOLLOWER_CONTROL_H

#include <stdint.h>

#include "line_sensor.h"
#include "current_sensor.h"
#include "hardware.h"

namespace line_follower {

class Control {
public:
  enum class State {
    kIdle,
    kWaitForReady,
    kReady,
    kOperational
  };
  
  struct ControlOutput {
    float motor_pwm[2];
    bool motor_enable;
    PistonState piston_state;
    State state; 
  };

  Control();
  void Reset();
  void Poll(uint32_t micros, ControlOutput *output);

  void TransitionToReady();
  void TransitionToOperational();
  void TransitionToIdle();
  void TransitionDown();
  void TransitionUp();

  void set_pid_kp(float pid_kp) { pid_kp_ = pid_kp; }
  float pid_kp() { return pid_kp_; }

  void set_pid_kd(float pid_kd) { pid_kd_ = pid_kd; }
  float pid_kd() { return pid_kd_; }

  float MaxCurrent_A();


private:  
  void RunStateMachine(uint32_t micros, ControlOutput *output);

  LineSensor line_sensor_ = {};
  CurrentSensor current_sensors_[kNumCurrentSensors] = {};

  State command_;
  State state_;
  State last_state_;
  uint32_t last_idle_micros_;
  uint32_t last_micros_;
  float last_error_;
  
  uint32_t operational_start_micros_;

  float pid_kp_;
  float pid_kd_;  
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_CONTROL_H
