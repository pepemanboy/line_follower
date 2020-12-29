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

#ifndef min
  #define min(a,b) ((a) < (b) ? (a) : (b))
#endif

constexpr int32_t kPistonReady_micros = 10000000;
constexpr float kBaseRate_PwmDc = 0.4;
constexpr float kMinRate_PwmDc = 0;
constexpr float kMaxRate_PwmDc = 0.7;
constexpr float kPidKp = 0.0012;
constexpr float kPidKd = 6;
constexpr float kLineCentered_mm = 62.5;
constexpr int32_t kObstaclePresentHold_micros = 1100000;
constexpr float kMinBattery_V = 11.5;

namespace line_follower {

Control::Control():
  obstacle_present_(kObstaclePresentHold_micros) {
  Reset();
  pid_kp_ = kPidKp;
  pid_kd_ = kPidKd;
}

void Control::Reset() {
  line_sensor_.Reset();
  battery_meter_.Reset();
  low_battery_ = false;

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

  // Read battery meter.
  battery_meter_.OnAnalogSample(ReadBatteryMeter());
  low_battery_ = battery_meter_.Output_V() <= kMinBattery_V;

  // Read range sensor.
  obstacle_present_.OnDigitalRead(micros, ReadRangeSensor());

  RunStateMachine(micros, output);
}

void Control::TransitionDown() {
  if (last_state_ > State::kIdleReady &&
      last_state_ != State::kError) {
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

bool Control:: IsLineSensorCentered() {
  const auto line_reading = line_sensor_.MaybeOutput_mm();
  return line_reading.valid && 
    abs(line_reading.value.average) < kLineCentered_mm;  
}

void Control::RunStateMachine(uint32_t micros, ControlOutput *output) {
  // Check for new state and update last state.
  const bool new_state = state_ != last_state_;
  output->state = last_state_;
  last_state_ = state_;

  // FSM.
  switch(state_) {
    case State::kIdle:      
      transition_to_operational_ = false;
      output->motor_enable = false;
      output->piston_state = PistonState::Up;
      last_idle_micros_ = micros;      
      
      if (IsLineSensorCentered()) {
        state_ = State::kIdleReady;
      }
      break;
    case State::kIdleReady:
      if (low_battery_) {
        state_ = State::kError;
        break;
      }
      output->motor_enable = false;
      output->piston_state = PistonState::Up;
      last_idle_micros_ = micros;      
      if (!IsLineSensorCentered()) {
        state_ = State::kIdle;
      } else if (transition_to_operational_) {
        state_ = State::kWaitForReady;
      }
      break;
    case State::kWaitForReady: {
      if (low_battery_) {
        state_ = State::kError;
        break;
      }
      output->motor_enable = false;
      output->piston_state = PistonState::Down;
      if (micros - last_idle_micros_ < kPistonReady_micros) {
        break;
      }
      if (IsLineSensorCentered()) {
        state_ = State::kReady;
      }
      break;
    }
    case State::kReady: {
      if (!IsLineSensorCentered()) {
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
        transition_to_operational_ = false;
        for (int i = 0; i < 2; ++i) {
          output->motor_pwm[0] = 0;
        }  
      }

      // Check for valid line reading.
      const MaybeValid<Stats> maybe_line = line_sensor_.MaybeOutput_mm();
      if (!maybe_line.valid) {
        state_ = State::kIdle;
        break;
      }

      // Check for obstacles.
      if (obstacle_present_.output()) {
        state_ = State::kOperationalPause;
        break;
      } 

      // Run PD controller.
      const float error = maybe_line.value.average;
      const float dt_s = (micros - last_micros_) / 1000000.0f;
      if (dt_s == 0) break;
      const float d_output = (error - last_error_) / dt_s * pid_kd_;
      const float pd_output = pid_kp_ * (error + d_output);
      last_error_ = error;

      // Update motor output.
      output->motor_pwm[0] = 
        ClampToRange(kBaseRate_PwmDc + pd_output, kMinRate_PwmDc, 
          kMaxRate_PwmDc);
      output->motor_pwm[1] =
        ClampToRange(kBaseRate_PwmDc - pd_output, kMinRate_PwmDc, 
          kMaxRate_PwmDc);
      output->motor_enable = true;

      break;
    }
    case State::kOperationalPause: {
      // Stop motors.
      for (int i = 0; i < 2; ++i) {
        output->motor_pwm[i] = 0;
      }

      // Check for obstacles.
      if (!obstacle_present_.output()) {
        state_ = State::kOperational;
        break;
      } 
      break;
    }
    case State::kError: {      
      // Unrecoverable.
      output->motor_enable = false;
      output->piston_state = PistonState::Up;
      break;
    }
  }
  
  last_micros_ = micros;    
}

}  // namespace line_follower