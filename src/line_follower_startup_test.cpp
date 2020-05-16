#include "line_follower_startup_test.h"

#include <stdint.h>

#include <Arduino.h>
#include "hardware.h"
#include "util.h"
#include "line_sensor.h"
#include "current_sensor.h"
#include "button_debounce.h"

namespace line_follower {

LineFollowerStartupTest::LineFollowerStartupTest() {}

#define Bluetooth Serial2

int TestPrompt(const char * prompt) {
  Bluetooth.print(prompt);
  Bluetooth.println(". Press any key (or q to quit).");
  while(!Bluetooth.available()) continue;
  int response = Bluetooth.read();
  char buf[20];
  sprintf(buf, "Received %c", response);
  Bluetooth.println(buf);
  return response;
}

#define TESTPROMPTRETURN(s) if(TestPrompt(s) == 'q') return

void LineFollowerStartupTest::StartupTest() {
  Bluetooth.println("Startup test!");
  TESTPROMPTRETURN("Blinkng red LED");
  SetLed(Led::Red, true);
  delay(1000);
  SetLed(Led::Red, false);

  TESTPROMPTRETURN("Blinkng green LED");
  SetLed(Led::Green, true);
  delay(1000);
  SetLed(Led::Green, false);

  TESTPROMPTRETURN("Blinking red tower light");
  SetTowerLight(TowerLight::Red, true);
  delay(1000);
  SetTowerLight(TowerLight::Red, false);

  TESTPROMPTRETURN("Blinking green tower light");
  SetTowerLight(TowerLight::Green, true);
  delay(1000);
  SetTowerLight(TowerLight::Green, false);

  TESTPROMPTRETURN("Blinking yellow tower light");
  SetTowerLight(TowerLight::Yellow, true);
  delay(1000);
  SetTowerLight(TowerLight::Yellow, false);

  TESTPROMPTRETURN("Quick tower sound");
  SetTowerSound(true);
  delay(500);
  SetTowerSound(false);

  TESTPROMPTRETURN("Piston up");
  SetPiston(PistonState::Up);
  delay(5000);
  SetPiston(PistonState::Idle);

  TESTPROMPTRETURN("Piston down");
  SetPiston(PistonState::Down);
  delay(5000);
  SetPiston(PistonState::Idle);

  TESTPROMPTRETURN("Left motor forward");
  EnableMotor(Motor::Left, true);
  SetMotorPwm(Motor::Left, 20);
  delay(5000);
  SetMotorPwm(Motor::Left, 0);
  EnableMotor(Motor::Left, false);

  TESTPROMPTRETURN("Left motor reverse");
  EnableMotor(Motor::Left, true);
  SetMotorPwm(Motor::Left, -20);
  delay(5000);
  SetMotorPwm(Motor::Left, 0);
  EnableMotor(Motor::Left, false);

  TESTPROMPTRETURN("Right motor forward");
  EnableMotor(Motor::Right, true);
  SetMotorPwm(Motor::Right, 20);
  delay(5000);
  SetMotorPwm(Motor::Right, 0);
  EnableMotor(Motor::Right, false);

  TESTPROMPTRETURN("Right motor reverse");
  EnableMotor(Motor::Right, true);
  SetMotorPwm(Motor::Right, -20);
  delay(5000);
  SetMotorPwm(Motor::Right, 0);
  EnableMotor(Motor::Right, false);

  while(!Serial.available()) {
    Bluetooth.println("QTR sensors:");
    int32_t qtr_sensors[kNumQtrSensors];
    ReadQtrSensors(qtr_sensors);
    for (int i = 0; i < kNumQtrSensors; ++i) {
      char buf[20];
      sprintf(buf, "QTR%d = %d", i, (int)qtr_sensors[i]);
      Bluetooth.println(buf);
    }

    Bluetooth.println("Current sensors:");
    int32_t current_sensors[kNumCurrentSensors];
    ReadCurrentSensors(current_sensors);
    for (int i = 0; i < kNumCurrentSensors; ++i) {
      char buf[20];
      sprintf(buf, "CS%d = %d", i, (int)current_sensors[i]);
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

  Serial.read();
}

void LineFollowerStartupTest::LineSensorsTest() {
  Bluetooth.println("Line sensors test!");
  LineSensor line_sensor = {};

  TESTPROMPTRETURN("Piston down");
  SetPiston(PistonState::Down);
  delay(10000);
  SetPiston(PistonState::Idle);

  while(!Serial.available()) {
    Bluetooth.println("QTR sensors:");
    int32_t qtr_sensors[kNumQtrSensors];
    ReadQtrSensors(qtr_sensors);
    line_sensor.OnQtrArrayReading(qtr_sensors);

    float raw_average = 0;   
    float raw_min = 2000; 
    for (int i = 0; i < kNumQtrSensors; ++i) {
      raw_average += qtr_sensors[i];
      raw_min = min(raw_min, qtr_sensors[i]);
      char buf[30];
      sprintf(buf, "QTR%d = %d, %d", i, 
              (int)qtr_sensors[i], 
              (int)line_sensor.DebugRawFilters()[i].output());
      Bluetooth.println(buf);
    }
    raw_average /= kNumQtrSensors;
    
    MaybeValid<Stats> line_sensor_stats = line_sensor.MaybeOutput_mm();
    char avg_float[10];
    dtostrf(line_sensor_stats.value.average, 7, 3, avg_float);
    char stddev_float[10];
    dtostrf(line_sensor_stats.value.std_dev, 7, 3, stddev_float);
    char raw_float[10];
    dtostrf(raw_average, 7, 3, raw_float);
    char min_float[10];
    dtostrf(raw_min, 7, 3, min_float);
    char buf[70];
    sprintf(buf, "Valid? %d Average: %s StdDev: %s Raw: %s Min %s",
            line_sensor_stats.valid, avg_float, stddev_float, 
            raw_float, min_float);
    Bluetooth.println(buf);

    Bluetooth.println();
    Bluetooth.println();
    delay(1000);
  }

  Serial.read();
}

void LineFollowerStartupTest::CurrentSensorsTest() {
  Bluetooth.println("Current sensors test!");
  CurrentSensor current_sensors[kNumCurrentSensors];

  TESTPROMPTRETURN("Piston down");
  SetPiston(PistonState::Down);
  delay(10000);
  SetPiston(PistonState::Idle);

  TESTPROMPTRETURN("Drive forward");
  EnableMotor(Motor::Left, true);
  EnableMotor(Motor::Right, true);
  SetMotorPwm(Motor::Left, 10);
  SetMotorPwm(Motor::Right, 10);  

  while(!Serial.available()) {
    Bluetooth.println("Current sensors:");
    int32_t current_readings[kNumCurrentSensors];
    ReadCurrentSensors(current_readings);
    for (int i = 0; i < kNumCurrentSensors; ++i) {
      current_sensors[i].OnAnalogSample(current_readings[i]);
    }

    for (int i = 0; i < kNumCurrentSensors; ++i) {
      char buf[30];
      sprintf(buf, "CS%d = %d, %d", i, 
              (int)current_readings[i], 
              (int)current_sensors[i].Output_Amps());
      Bluetooth.println(buf);
    }

    Bluetooth.println();
    Bluetooth.println();
    delay(1000);
  }

  EnableMotor(Motor::Left, false);
  EnableMotor(Motor::Right, false);
  Serial.read();
}

void LineFollowerStartupTest::ButtonsTest() {
  Bluetooth.println("Buttons test!");
  Bluetooth.println("Press a button.");

  ButtonDebounce button_up = {};
  ButtonDebounce button_down = {};

  while(!Serial.available()) {
    const uint32_t now = micros();

    // Read buttons.
    button_up.OnDigitalRead(now, ReadButton(Button::Up));    
    button_down.OnDigitalRead(now, ReadButton(Button::Down));

    if (button_up.Pulse()) {
      Bluetooth.println("Button up pressed");
    }
    if (button_down.Pulse()) {
      Bluetooth.println("Button down pressed");
    }

    delay(100);
  }
  Serial.read();
}

void LineFollowerStartupTest::Init() {
  HardwareInit();
  Bluetooth.begin(9600);
}

void LineFollowerStartupTest::Poll(uint32_t micros) {
  Bluetooth.println();
  Bluetooth.println();
  Bluetooth.println("Welcome to the test");
  Bluetooth.println("Startup test: Press 1");
  Bluetooth.println("Line sensors test: Press 2");
  Bluetooth.println("Current sensors test: Press 3");
  Bluetooth.println("Buttons test: Press 4");
  int option = TestPrompt("Choose a test") - (int)'0';

  switch(option) {
    case 1: StartupTest(); break;
    case 2: LineSensorsTest(); break;
    case 3: CurrentSensorsTest(); break;
    case 4: ButtonsTest(); break;
    default:
      Bluetooth.println("Invalid option");
      break;
  }
}

}  // namespace line_follower
