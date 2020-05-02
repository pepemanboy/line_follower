#ifndef LINE_FOLLOWER_PINOUT_H
#define LINE_FOLLOWER_PINOUT_H

#include <Arduino.h>

#include "constants.h"

namespace line_follower {

constexpr int kQtrSensorPins[kNumQtrSensors] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13
};

constexpr int kCurrentSensorPins[kNumCurrentSensors] = {
  A15, A14
};

constexpr int kTowerLightRedPin = 45;
constexpr int kTowerLightGreenPin = 43;
constexpr int kTowerLightYellowPin = 41;
constexpr int kTowerSoundPin = 39;

constexpr int kButtonUpPin = 51;
constexpr int kButtonDownPin = 53;

constexpr int kPistonPlusPin = 37;
constexpr int kPistonMinusPin = 35;

constexpr int kMotorDriver1LeftPwmPin = 73;
constexpr int kMotorDriver1RightPwmPin = 72;
constexpr int kMotorDriver1LeftEnablePin = 74;
constexpr int kMotorDriver1RightEnablePin = 71;

constexpr int kMotorDriver2LeftPwmPin = 75;
constexpr int kMotorDriver2RightPwmPin = 78;
constexpr int kMotorDriver2LeftEnablePin = 76;
constexpr int kMotorDriver2RightEnablePin = 77;

constexpr int kLedGreenPin = 29;
constexpr int kLedRedPin = 27;


} // namespace line_follower


#endif  // LINE_FOLLOWER_PINOUT_H
