#include "hardware.h"

#include <stdint.h>

#include <Arduino.h>
#include "pinout.h"

namespace line_follower {

void HardwareInit() {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    pinMode(kQtrSensorPins[i], INPUT);
  }

  for (int i = 0; i < kNumCurrentSensors; ++i) {
    pinMode(kCurrentSensorPins[i], INPUT);
  }

  pinMode(kLedGreenPin, OUTPUT);
  pinMode(kLedRedPin, OUTPUT);
  SetLed(Led::Green, false);
  SetLed(Led::Red, false);

  pinMode(kTowerLightRedPin, OUTPUT);
  pinMode(kTowerLightGreenPin, OUTPUT);
  pinMode(kTowerLightYellowPin, OUTPUT);
  pinMode(kTowerSoundPin, OUTPUT);

  SetTowerLight(TowerLight::Green, false);
  SetTowerLight(TowerLight::Red, false);
  SetTowerLight(TowerLight::Yellow, false);
  SetTowerSound(false);

  pinMode(kButtonUpPin, INPUT);
  pinMode(kButtonDownPin, INPUT);

  pinMode(kPistonPlusPin, OUTPUT);
  pinMode(kPistonPlusPin, OUTPUT);
  SetPiston(PistonState::Idle);

  pinMode(kMotorDriver1LeftPwmPin, OUTPUT);
  pinMode(kMotorDriver1RightPwmPin, OUTPUT);
  pinMode(kMotorDriver1LeftEnablePin, OUTPUT);
  pinMode(kMotorDriver1RightEnablePin, OUTPUT);
  EnableMotor(Motor::Left, false);

  pinMode(kMotorDriver2LeftPwmPin, OUTPUT);
  pinMode(kMotorDriver2RightPwmPin, OUTPUT);
  pinMode(kMotorDriver2LeftEnablePin, OUTPUT);
  pinMode(kMotorDriver2RightEnablePin, OUTPUT);
  EnableMotor(Motor::Right, false);
}

void ReadQtrSensors(int readings[kNumQtrSensors]) {
  for (int i = 0; i < kNumQtrSensors; ++i) {
    readings[i] = analogRead(kQtrSensorPins[i]);
  }
}

void ReadCurrentSensors(int readings[kNumCurrentSensors]) {
  for (int i = 0; i < kNumCurrentSensors; ++i) {
    readings[i] = analogRead(kCurrentSensorPins[i]);
  }
}

bool ReadButton(Button button) {
  switch(button) {
    case Button::Up: return digitalRead(kButtonUpPin);
    case Button::Down: return digitalRead(kButtonDownPin);
  }
  return false;  // Shall never get here.
}

void SetTowerLight(TowerLight light, bool state) {
  switch (light) {
    case TowerLight::Red: digitalWrite(kTowerLightRedPin, !state); break;
    case TowerLight::Green: digitalWrite(kTowerLightGreenPin, !state); break;
    case TowerLight::Yellow: digitalWrite(kTowerLightYellowPin, !state); break;
  }
}

void SetTowerSound(bool state) {
  digitalWrite(kTowerSoundPin, !state);
}

void SetPiston(PistonState state) {
  switch(state) {
    case PistonState::Up:
      digitalWrite(kPistonPlusPin, HIGH);
      digitalWrite(kPistonMinusPin, LOW);
      break;
    case PistonState::Down:
      digitalWrite(kPistonPlusPin, LOW);
      digitalWrite(kPistonMinusPin, HIGH);
      break;
    case PistonState::Idle:
      digitalWrite(kPistonPlusPin, LOW);
      digitalWrite(kPistonPlusPin, LOW);
  }
}

void SetMotorPwm(Motor motor, int pwm) {
  int forward_pwm_pin, reverse_pwm_pin;
  switch(motor) {
    case Motor::Left:
      forward_pwm_pin = kMotorDriver1LeftPwmPin;
      reverse_pwm_pin = kMotorDriver1RightPwmPin;
      break;
    case Motor::Right:
      forward_pwm_pin = kMotorDriver2RightPwmPin;
      reverse_pwm_pin = kMotorDriver2LeftPwmPin;
      break;
  }

  if (pwm >= 0) {
    analogWrite(forward_pwm_pin, pwm);
    analogWrite(reverse_pwm_pin, 0);
  } else {
    analogWrite(reverse_pwm_pin, -pwm);
    analogWrite(forward_pwm_pin, 0);
  }
}

void EnableMotor(Motor motor, bool enable) {
  int forward_enable_pin, reverse_enable_pin;
  switch(motor) {
    case Motor::Left:
      forward_enable_pin = kMotorDriver1LeftEnablePin;
      reverse_enable_pin = kMotorDriver1RightEnablePin;
      break;
    case Motor::Right:
      forward_enable_pin = kMotorDriver2RightEnablePin;
      reverse_enable_pin = kMotorDriver2LeftEnablePin;
      break;
  }
  digitalWrite(forward_enable_pin, enable);
  digitalWrite(reverse_enable_pin, enable);
}

void SetLed(Led led, bool state) {
  switch(led) {
    case Led::Green: digitalWrite(kLedGreenPin, state); break;
    case Led::Red: digitalWrite(kLedRedPin, state); break;
  }
}






}  // namespace line_follower
