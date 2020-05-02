#include "line_follower_startup_test.h"

#include <stdint.h>

#include <Arduino.h>
#include "hardware.h"

namespace line_follower {

LineFollowerStartupTest::LineFollowerStartupTest() {}

#define Bluetooth Serial2

void TestPrompt(const char * prompt) {
  Bluetooth.print(prompt);
  Bluetooth.println("! Ready? Press any key.");
  while(!Bluetooth.available()) continue;
  while(Bluetooth.available()) {
    Bluetooth.read();
    delay(10);
  }
  Bluetooth.println("OK");
}

void LineFollowerStartupTest::Init() {
  HardwareInit();

  Bluetooth.begin(9600);

  TestPrompt("Line follower startup test");

  TestPrompt("Blinkng red LED");
  SetLed(Led::Red, true);
  delay(1000);
  SetLed(Led::Red, false);

  TestPrompt("Blinkng green LED");
  SetLed(Led::Green, true);
  delay(1000);
  SetLed(Led::Green, false);

  TestPrompt("Blinking red tower light");
  SetTowerLight(TowerLight::Red, true);
  delay(1000);
  SetTowerLight(TowerLight::Red, false);

  TestPrompt("Blinking green tower light");
  SetTowerLight(TowerLight::Green, true);
  delay(1000);
  SetTowerLight(TowerLight::Green, false);

  TestPrompt("Blinking yellow tower light");
  SetTowerLight(TowerLight::Yellow, true);
  delay(1000);
  SetTowerLight(TowerLight::Yellow, false);

  TestPrompt("Quick tower sound");
  SetTowerSound(true);
  delay(500);
  SetTowerSound(false);

  TestPrompt("Piston up");
  SetPiston(PistonState::Up);
  delay(3000);

  TestPrompt("Piston down");
  SetPiston(PistonState::Down);
  delay(3000);
  SetPiston(PistonState::Idle);

  TestPrompt("Left motor forward");
  EnableMotor(Motor::Left, true);
  SetMotorPwm(Motor::Left, 20);
  delay(3000);

  TestPrompt("Left motor reverse");
  SetMotorPwm(Motor::Left, -20);
  delay(3000);
  SetMotorPwm(Motor::Left, 0);
  EnableMotor(Motor::Left, false);

  TestPrompt("Right motor forward");
  EnableMotor(Motor::Right, true);
  SetMotorPwm(Motor::Right, 20);
  delay(3000);

  TestPrompt("Right motor reverse");
  SetMotorPwm(Motor::Right, -20);
  delay(3000);
  SetMotorPwm(Motor::Right, 0);
  EnableMotor(Motor::Right, false);
}

void LineFollowerStartupTest::Poll(uint32_t micros) {
  Bluetooth.println("QTR sensors:");
  int qtr_sensors[kNumQtrSensors];
  ReadQtrSensors(qtr_sensors);
  for (int i = 0; i < kNumQtrSensors; ++i) {
    char buf[20];
    sprintf(buf, "QTR%d = %d", i, qtr_sensors[i]);
    Bluetooth.println(buf);
  }

  Bluetooth.println("Current sensors:");
  int current_sensors[kNumCurrentSensors];
  ReadCurrentSensors(current_sensors);
  for (int i = 0; i < kNumCurrentSensors; ++i) {
    char buf[20];
    sprintf(buf, "CS%d = %d", i, current_sensors[i]);
    Bluetooth.println(buf);
  }

  Bluetooth.println("Buttons:");
  {
    char buf[20];
    sprintf(buf, "Up %d, down %d",
            ReadButton(Button::Up),
            ReadButton(Button::Down));
    Bluetooth.println(buf);
  }

  Bluetooth.println();
  Bluetooth.println();
  delay(1000);
}

}  // namespace line_follower
