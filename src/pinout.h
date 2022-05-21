#ifndef LINE_FOLLOWER_PINOUT_H
#define LINE_FOLLOWER_PINOUT_H

#include <Arduino.h>

#include "constants.h"

namespace line_follower {

constexpr int kQtrSensorPins[kNumQtrSensors] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13
};

constexpr int kBatteryMeterPin = A14;

constexpr int kTowerLightRedPin = 45;
constexpr int kTowerLightGreenPin = 43;
constexpr int kTowerLightYellowPin = 41;
constexpr int kTowerSoundPin = 39;

constexpr int kButtonDownPin = 51;
constexpr int kButtonUpPin = 53;

constexpr int kRangeSensorPin = 49;

constexpr int kPistonPlusPin = 37;
constexpr int kPistonMinusPin = 35;

// MotorDriver 1 is left.
constexpr int kMotorDriver1LeftPwmPin = 4;
constexpr int kMotorDriver1RightPwmPin = 3;
constexpr int kMotorDriver1LeftEnablePin = 5;
constexpr int kMotorDriver1RightEnablePin = 2;

// MotorDriver2 is right.
constexpr int kMotorDriver2LeftPwmPin = 6;
constexpr int kMotorDriver2RightPwmPin = 9;
constexpr int kMotorDriver2LeftEnablePin = 7;
constexpr int kMotorDriver2RightEnablePin = 8;

constexpr int kLedGreenPin = 29;
constexpr int kLedRedPin = 27;

} // namespace line_follower


#endif  // LINE_FOLLOWER_PINOUT_H
