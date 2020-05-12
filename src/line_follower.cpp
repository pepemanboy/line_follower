#include "line_follower.h"

#include <stdint.h>

#include <Arduino.h>

#include "control.h"

namespace line_follower {

LineFollower::LineFollower() {

}

void LineFollower::Init() {
  HardwareInit();
  control_.Reset();
}

void LineFollower::Poll(uint32_t micros) {
  Control::ControlOutput control_output;
  if (ReadButton(Button::Down)) {
    control_.TransitionToReady();
  }
  if (ReadButton(Button::Up)) {
    control_.TransitionToOperational();
  }

  control_.Poll(micros, &control_output);

  // Update tower light and sound.
  UpdateTower(micros, control_output.state);
  
}

void LineFollower::UpdateTower(uint32_t micros, Control::State state) {
  static uint32_t last_toggle_micros = micros;
  static uint32_t toggle_state = true;
  if (micros - last_toggle_micros > 500000) {
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
