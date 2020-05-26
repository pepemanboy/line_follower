#include "control.h"
#include "hardware.h"
#include "util.h"

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif
#define abs(x) ((x)>0?(x):-(x))

#ifndef max
  #define max(a,b) ((a) > (b) ? (a) : (b))
#endif

constexpr int32_t kPistonReady_micros = 10000000;
constexpr float kBasePwmDc = 0.5;
constexpr float kMinPwmDc = 0;
constexpr float kMaxPwmDc = 0.8;
constexpr float kMaxCurrent_A = 1000;
constexpr float kPidKp = 0.0012;
constexpr float kPidKd = 6;
constexpr int32_t kMotorStartUp_micros = 1000000;

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

  transition_to_operational_ = false;
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

void Control::TransitionDown() {
  if (last_state_ > State::kIdleReady) {
    state_ = State::kIdle;
  }
}

void Control::TransitionUp() {
  if (last_state_ == State::kIdleReady) {
    state_ = State::kWaitForReady;
  } else if (last_state_ == State::kReady) {
    state_ = State::kOperational;
  }
  return;
}

void Control::TransitionToOperational() {
  transition_to_operational_ = true;
}

void Control::RunStateMachine(uint32_t micros, ControlOutput *output) {
  const bool new_state = state_ != last_state_;
  switch(state_) {
    case State::kIdle:
      transition_to_operational_ = false;
      output->motor_enable = false;
      output->piston_state = PistonState::Up;
      last_idle_micros_ = micros;
      if (line_sensor_.MaybeOutput_mm().valid) {
        state_ = State::kIdleReady;
      }
      break;
    case State::kIdleReady:
      output->motor_enable = false;
      output->piston_state = PistonState::Up;
      last_idle_micros_ = micros;
      if (!line_sensor_.MaybeOutput_mm().valid) {
        state_ = State::kIdle;
      } else if (transition_to_operational_) {
        state_ = State::kWaitForReady;
      }
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
        transition_to_operational_ = false;
      } else if (transition_to_operational_) {
        state_ = State::kOperational;
      }
      break;
    }
    case State::kOperational: {
      if (new_state) {
        last_error_ = 0;
        operational_start_micros_ = micros;
        transition_to_operational_ = false;
      }

      const uint32_t micros_since_operational_start =
        micros - operational_start_micros_; 
      const bool motor_startup_finished = micros_since_operational_start < 
        kMotorStartUp_micros;

      // Check for valid line reading.
      const MaybeValid<Stats> maybe_line = line_sensor_.MaybeOutput_mm();
      if (!maybe_line.valid) {
        state_ = State::kIdle;
        break;
      }
      
      // Overcurrent / torque protection.
      if (motor_startup_finished) {
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
      }      

      // Run PD controller.
      const float error = maybe_line.value.average;
      const float dt_s = (micros - last_micros_) / 1000000.0f;
      if (dt_s == 0) break;
      const float d_output = (error - last_error_) / dt_s * pid_kd_;
      const float pd_output = pid_kp_ * (error + d_output);
      last_error_ = error;

      output->motor_pwm[0] = 
        ClampToRange(kBasePwmDc + pd_output, kMinPwmDc, kMaxPwmDc);
      output->motor_pwm[1] =
        ClampToRange(kBasePwmDc - pd_output, kMinPwmDc, kMaxPwmDc);
      output->motor_enable = true;

      // Ramp up on motor start up.
      if (!motor_startup_finished) {
        const float ramp = micros_since_operational_start / 
          (float) kMotorStartUp_micros;
        for (int i = 0; i < 2; ++i) {
          output->motor_pwm[i] *= ramp;
        }
      }

      break;
    }
  }

  // Update.
  output->state = last_state_;
  last_state_ = state_;
  last_micros_ = micros;    
}

float Control::MaxCurrent_A() {
  float max_current_A = 0;
  for (int i = 0; i < kNumCurrentSensors; ++i) {
    max_current_A = max(max_current_A, current_sensors_[i].Output_Amps());
  }
  return max_current_A;
}

}  // namespace line_follower