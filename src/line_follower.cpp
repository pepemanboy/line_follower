#include "line_follower.h"

#include <stdint.h>

#include <Arduino.h>

#include "control.h"

constexpr uint32_t kTowerBlinkPeriod_micros = 500000;

namespace line_follower {

LineFollower::LineFollower() {

}

void LineFollower::Init() {
  HardwareInit();
  control_.Reset();
}

void LineFollower::Poll(uint32_t micros) {
  // Read buttons.
  button_up_.OnDigitalRead(micros, ReadButton(Button::Up));  
  if (button_up_.Pulse()) control_.TransitionUp();
  button_down_.OnDigitalRead(micros, ReadButton(Button::Down));
  if (button_down_.Pulse()) control_.TransitionDown();

  // Run control cycle.
  Control::ControlOutput control_output;
  control_.Poll(micros, &control_output);

  // Update tower light and sound.
  UpdateTower(micros, control_output.state);

  // Update motors.
  EnableMotor(Motor::Left, control_output.motor_enable);
  EnableMotor(Motor::Right, control_output.motor_enable);
  SetMotorPwm(Motor::Left, control_output.motor_pwm[0]);
  SetMotorPwm(Motor::Right, control_output.motor_pwm[1]);

  // Update piston.
  SetPiston(control_output.piston_state);
}

void LineFollower::UpdateTower(uint32_t micros, Control::State state) {
  static uint32_t last_toggle_micros = micros;
  static bool toggle_state = true;
  if (micros - last_toggle_micros > kTowerBlinkPeriod_micros) {
    last_toggle_micros = micros;
    toggle_state = !toggle_state;
  }
  bool green = false, red = false, yellow = false, sound = false;
  switch(state) {
    case Control::State::kIdle:  
      green = true;
      break;
    case Control::State::kWaitForReady:     
      yellow = true;
      break;
    case Control::State::kReady:
      yellow = toggle_state;
      break;
    case Control::State::kOperational:
      red = toggle_state;
      sound = toggle_state;
      break;
  }
  SetTowerLight(TowerLight::Red, red);
  SetTowerLight(TowerLight::Green, green);
  SetTowerLight(TowerLight::Yellow, yellow);
  SetTowerSound(sound);
}

}  // namespace line_follower
