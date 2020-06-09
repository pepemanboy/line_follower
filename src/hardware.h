#ifndef LINE_FOLLOWER_HARDWARE_H
#define LINE_FOLLOWER_HARDWARE_H

#include <stdint.h>

#include "constants.h"

namespace line_follower {

enum class TowerLight {
  Red,
  Green,
  Yellow
};

enum class Button {
  Up,
  Down
};

enum class PistonState {
  Up,
  Down,
  Idle
};

enum class Motor {
  Left,
  Right,
};

enum class Led {
  Green,
  Red
};

void HardwareInit();

void ReadQtrSensors(int32_t readings[kNumQtrSensors]);
void ReadCurrentSensors(int32_t readings[kNumCurrentSensors]);
bool ReadButton(Button button);
bool ReadRangeSensor();

void SetTowerLight(TowerLight light, bool state);
void SetTowerSound(bool state);

void SetPiston(PistonState state);

void SetMotorPwm(Motor motor, int pwm);
void EnableMotor(Motor motor, bool enable);

void SetLed(Led led, bool state);

}  // namespace line_follower

#endif  // LINE_FOLLOWER_HARDWARE_H
