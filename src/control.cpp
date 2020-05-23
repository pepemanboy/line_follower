#include "control.h"
#include "hardware.h"
#include "util.h"

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif
#define abs(x) ((x)>0?(x):-(x))

constexpr int32_t kPistonReady_micros = 10000000;
constexpr float kBasePwmDc = 0.5;
constexpr float kMinPwmDc = 0;
constexpr float kMaxPwmDc = 0.9;
constexpr float kMaxCurrent_A = 1000;
constexpr float kPidKp = 0.001;
constexpr float kPidKd = 7;

namespace line_follower {

Control::Control() {
  Reset();
  pid_kp_ = kPidKp;
  pid_kd_ = kPidKd;
}

void Control::Reset() {
  line_sensor_.Reset();
  for(int i = 0; i < kNumCurrentSensors; ++i) {
    current_sensors_[i].Reset();
  }
  state_ = State::kIdle;
  last_state_ = State::kIdle;
  last_idle_micros_ = 0;
  last_micros_ = 0;
  last_error_ = 0;
}

void Control::Poll(uint32_t micros, ControlOutput *output) {
  // Read line sensors.
  int32_t qtr_readings[kNumQtrSensors];
  ReadQtrSensors(qtr_readings);
  line_sensor_.OnQtrArrayReading(qtr_readings);

  // Read current sensors.
  int32_t current_readings[kNumCurrentSensors];
  ReadCurrentSensors(current_readings);
  for (int i = 0; i < kNumCurrentSensors; ++i) {
    current_sensors_[i].OnAnalogSample(current_readings[i]);
  }

  RunStateMachine(micros, output);
}

void Control::TransitionToReady() {
  if (last_state_ == State::kIdle) {
    state_ = State::kWaitForReady;
  }
}

void Control::TransitionToOperational() {
  if (last_state_ == State::kReady) {
    state_ = State::kOperational;
  }
}

void Control::TransitionToIdle() {
  if (last_state_ != State::kIdle) {
    state_ = State::kIdle;
  }
}

void Control::TransitionDown() {
  TransitionToIdle();
}

void Control::TransitionUp() {
  if (last_state_ == State::kIdle) return TransitionToReady();
  TransitionToOperational();
}

void Control::RunStateMachine(uint32_t micros, ControlOutput *output) {
  const bool new_state = state_ != last_state_;
  switch(state_) {
    case State::kIdle:
       output->motor_enable = false;
       output->piston_state = PistonState::Up;
       last_idle_micros_ = micros;
       break;
    case State::kWaitForReady: {
      output->motor_enable = false;
      output->piston_state = PistonState::Down;
      if (micros - last_idle_micros_ < kPistonReady_micros) {
        break;
      }
      if (line_sensor_.MaybeOutput_mm().valid) {
        state_ = State::kReady;
      }
      break;
    }
    case State::kReady: {
      const MaybeValid<Stats> maybe_line = line_sensor_.MaybeOutput_mm();
      if (!maybe_line.valid) {
        state_ = State::kWaitForReady;
        break;
      }
      break;
    }
    case State::kOperational: {
      if (new_state) {
        last_error_ = 0;
        ramp_mult_ = 0;
      }

      ramp_mult_ = ramp_mult_ + 0.01 > 1 ? 1 : ramp_mult_ + 0.01;

      // Check for valid line reading.
      const MaybeValid<Stats> maybe_line = line_sensor_.MaybeOutput_mm();
      if (!maybe_line.valid) {
        state_ = State::kIdle;
        break;
      }

      // Overcurrent / torque protection.
      bool overcurrent = false;
      for (int i = 0; i < kNumCurrentSensors; ++i) {
        if (abs(current_sensors_[i].Output_Amps()) >= kMaxCurrent_A) {
          overcurrent = true;
        }
      }
      if (overcurrent) {
        state_ = State::kIdle;
        break;
      }

      // Run PD controller.
      const float error = maybe_line.value.average;
      const float dt_s = (micros - last_micros_) / 1000000.0f;
      if (dt_s == 0) break;
      const float d_output = (error - last_error_) / dt_s * pid_kd_;
      const float pd_output = pid_kp_ * (error + d_output);

      output->motor_pwm[0] = 
        ClampToRange(kBasePwmDc + pd_output, kMinPwmDc, kMaxPwmDc) * ramp_mult_;
      output->motor_pwm[1] =
        ClampToRange(kBasePwmDc - pd_output, kMinPwmDc, kMaxPwmDc) * ramp_mult_;
      output->motor_enable = true;

      last_error_ = error;

      break;
    }
  }

  // Update.
  last_state_ = state_;
  last_micros_ = micros;  

  output->state = state_;
}

}  // namespace line_follower