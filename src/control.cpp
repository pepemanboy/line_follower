#include "control.h"
#include "hardware.h"
#include "util.h"
#include "Arduino.h"

#define Bluetooth Serial2

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
constexpr float kBaseRate_PwmDc = 0.5f;
constexpr float kMinRate_PwmDc = 0.0f;
constexpr float kMaxRate_PwmDc = 0.8f;
constexpr float kMaxAccel_PwmDc_s = 0.5f; 
constexpr float kMaxCurrent_A = 20.0f;
constexpr float kPidKp = 0.0012f;
constexpr float kPidKd = 6.0f;
constexpr float kLineCentered_mm = 62.5f;
constexpr float kObstaclePresentHold_micros = 1100000;

namespace line_follower {

Control::Control():
  obstacle_present_(kObstaclePresentHold_micros) {
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

  // Read range sensor.
  obstacle_present_.OnDigitalRead(micros, ReadRangeSensor());

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

bool Control:: IsLineSensorCentered() {
  const auto line_reading = line_sensor_.MaybeOutput_mm();
  return line_reading.valid && 
    abs(line_reading.value.average) < kLineCentered_mm;  
}

void Control::RunStateMachine(uint32_t micros, ControlOutput *output) {
  const bool new_state = state_ != last_state_;
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
          output->motor_pwm[0] = 0.0f;
        }      
      }

      // Check for valid line reading.
      const MaybeValid<Stats> maybe_line = line_sensor_.MaybeOutput_mm();
      if (!maybe_line.valid) {
        char float_buf[10];
        dtostrf(maybe_line.value.std_dev, 7, 3, float_buf);
        char buf[30];
        sprintf(buf, "Line error %s", float_buf);            
        Bluetooth.println(buf);
        state_ = State::kIdle;
        break;
      }
      
      // Overcurrent protection.
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

      // Check for obstacles.
      if (obstacle_present_.output()) {
        state_ = State::kOperationalPause;
        break;
      } 

      // Run PD controller.
      const float error = maybe_line.value.average;
      const float dt_s = (micros - last_micros_)  * 1e-6f;
      if (dt_s == 0.0f) break;
      const float d_output = (error - last_error_) / dt_s * pid_kd_;
      const float pd_output = pid_kp_ * (error + d_output);
      last_error_ = error;

      // Update motor output.
      const float max_pwm_delta = kMaxAccel_PwmDc_s * dt_s; // Limit accel.
      output->motor_pwm[0] = 
        ClampToRange(kBaseRate_PwmDc + pd_output, kMinRate_PwmDc, 
          min(kMaxRate_PwmDc, output->motor_pwm[0] + max_pwm_delta));
      output->motor_pwm[1] =
        ClampToRange(kBaseRate_PwmDc - pd_output, kMinRate_PwmDc, 
          min(kMaxRate_PwmDc, output->motor_pwm[1] + max_pwm_delta));
      output->motor_enable = true;

      break;
    }
    case State::kOperationalPause: {
      // Stop motors.
      for (int i = 0; i < 2; ++i) {
        output->motor_pwm[0] = 0.0f;
      }

      // Check for obstacles.
      if (!obstacle_present_.output()) {
        state_ = State::kOperational;
        break;
      } 
    }
  }

  // Update.
  output->state = last_state_;
  last_state_ = state_;
  last_micros_ = micros;    
}

}  // namespace line_follower