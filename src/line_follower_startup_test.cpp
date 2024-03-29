#include "line_follower_startup_test.h"

#include <stdint.h>

#include <Arduino.h>
#include "hardware.h"
#include "util.h"
#include "line_sensor.h"
#include "button_debounce.h"
#include "line_follower.h"
#include "battery_meter.h"

#define Bluetooth Serial2

namespace line_follower {

LineFollowerStartupTest::LineFollowerStartupTest() {}

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

  while(!Bluetooth.available()) {
    Bluetooth.println("QTR sensors:");
    int32_t qtr_sensors[kNumQtrSensors];
    ReadQtrSensors(qtr_sensors);
    for (int i = 0; i < kNumQtrSensors; ++i) {
      char buf[20];
      sprintf(buf, "QTR%d = %d", i, (int)qtr_sensors[i]);
      Bluetooth.println(buf);
    }

    Bluetooth.println("Battery meter:");
    int battery_meter_reading = ReadBatteryMeter();
    char buf[20];
    sprintf(buf, "VBat. %d ADC", battery_meter_reading);
    Bluetooth.println(buf);

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

  Bluetooth.read();
}

void LineFollowerStartupTest::LineSensorsTest() {
  Bluetooth.println("Line sensors test!");
  LineSensor line_sensor = {};

  Bluetooth.println("If you want piston down, press '1'. If not, other key.");
  while(!Bluetooth.available()) {
    delay(100);
  }
  char read = Bluetooth.read();

  if (read == '1') {
    Bluetooth.println("Piston going down");
    SetPiston(PistonState::Down);
    delay(10000);
  } else {
    Bluetooth.println("Piston staying up");
    SetPiston(PistonState::Up);
  }  

  float raw_min = 2000;
  float raw_max = 0;

  while(!Bluetooth.available()) {
    Bluetooth.println("QTR sensors:");
    int32_t qtr_sensors[kNumQtrSensors];
    ReadQtrSensors(qtr_sensors);
    line_sensor.OnQtrArrayReading(qtr_sensors);

    float raw_average = 0;
    for (int i = 0; i < kNumQtrSensors; ++i) {
      raw_average += qtr_sensors[i];
      raw_min = min(raw_min, qtr_sensors[i]);
      raw_max = max(raw_max, qtr_sensors[i]);
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
    char max_float[10];
    dtostrf(raw_max, 7, 3, max_float);
    char buf[80];
    sprintf(buf, "Valid? %d Average: %s StdDev: %s Raw: %s Min %s Max %s",
            line_sensor_stats.valid, avg_float, stddev_float, 
            raw_float, min_float, max_float);
    Bluetooth.println(buf);

    Bluetooth.println();
    Bluetooth.println();
    delay(1000);
  }
  Bluetooth.read();

  SetPiston(PistonState::Up);
}

void LineFollowerStartupTest::ButtonsTest() {
  Bluetooth.println("Buttons test!");
  Bluetooth.println("Press a button.");

  ButtonDebounce button_up = {};
  ButtonDebounce button_down = {};

  while(!Bluetooth.available()) {
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
  Bluetooth.read();
}

void LineFollowerStartupTest::LineFollowerTest() {
  Bluetooth.println("Line follower test!");
  Bluetooth.println("Press Up button to go operational");
  Bluetooth.println("Press Down button to idle");

  LineFollower robot;
  robot.Init();
  if (pid_kp_ >= 0) robot.SetControlPidKp(pid_kp_);
  if (pid_kd_ >= 0) robot.SetControlPidKd(pid_kd_);

  {
    char float_buf[20];
    dtostrf(robot.ControlPidKp(), 7, 4, float_buf);
    char buf[30];
    sprintf(buf, "KP = %s", float_buf);
    Bluetooth.println(buf);
  }

  {
    char float_buf[20];
    dtostrf(robot.ControlPidKd(), 7, 4, float_buf);
    char buf[30];
    sprintf(buf, "KD = %s", float_buf);
    Bluetooth.println(buf);
  }  

  while(!Bluetooth.available()) {
    robot.Poll(micros());
  }
  
  Bluetooth.read();
}

float LineFollowerStartupTest::ReadBluetoothFloat() {
  constexpr uint8_t buf_size = 10;
  uint8_t buf_index = 0;
  char buf[buf_size] = {0};

  while(buf_index < buf_size - 1) {
    if (Bluetooth.available()) {
      char read = Bluetooth.read();          
      Bluetooth.print(read);
      if (read == '\n' || read == '\r') break;
      buf[buf_index++] = read;  
    }
  }
  delay(100);
  if (Bluetooth.available()) {
    Bluetooth.read();
  }

  return atof(buf);
}

int LineFollowerStartupTest::ReadBluetoothInt() {
  constexpr uint8_t buf_size = 10;
  uint8_t buf_index = 0;
  char buf[buf_size] = {0};

  while(buf_index < buf_size - 1) {
    if (Bluetooth.available()) {
      char read = Bluetooth.read();          
      Bluetooth.print(read);
      if (read == '\n' || read == '\r') break;
      buf[buf_index++] = read;  
    }
  }
  delay(100);
  if (Bluetooth.available()) {
    Bluetooth.read();
  }

  return atoi(buf);
}

void LineFollowerStartupTest::AdjustPidKp() {
  Bluetooth.println("Send PID Kp followed by Enter");
  float new_kp = ReadBluetoothFloat();

  char new_kp_str[20];
  dtostrf(new_kp, 7, 4, new_kp_str);
  char buf[50];
  sprintf(buf, "New value is: %s, press '1' for OK", new_kp_str);
  Bluetooth.println(buf);

  while(!Bluetooth.available()) {
    delay(100);
  }
  char read = Bluetooth.read();

  if (read == '1') {
    pid_kp_ = new_kp;
    Bluetooth.println("Saved");
  } else {
    Bluetooth.println("Not saved");
  }
}

void LineFollowerStartupTest::AdjustPidKd() {
  Bluetooth.println("Send PID Kd followed by Enter");
  float new_kd = ReadBluetoothFloat();

  char new_kd_str[20];
  dtostrf(new_kd, 7, 4, new_kd_str);
  char buf[50];
  sprintf(buf, "New value is: %s, press '1' for OK", new_kd_str);
  Bluetooth.println(buf);

  while(!Bluetooth.available()) {
    delay(100);
  }
  char read = Bluetooth.read();

  if (read == '1') {
    pid_kd_ = new_kd;
    Bluetooth.println("Saved");
  } else {
    Bluetooth.println("Not saved");
  }
}

void LineFollowerStartupTest::RangeSensorTest() {
  Bluetooth.println("Range sensor test!");
  Bluetooth.println("Green light if OK, Red light if obstacle present");

  SampleHold obstacle_present(1100000);

  while(!Bluetooth.available()) {
    const bool range_sensor = ReadRangeSensor();
    obstacle_present.OnDigitalRead(micros(), range_sensor);
    if (obstacle_present.output()) {
      SetTowerLight(TowerLight::Green, false);
      SetTowerLight(TowerLight::Red, true);
    } else {
      SetTowerLight(TowerLight::Green, true);
      SetTowerLight(TowerLight::Red, false);
    }
    SetTowerLight(TowerLight::Yellow, range_sensor);  // Raw sensor.
    delay(10);
  }
  Bluetooth.read();

  SetTowerLight(TowerLight::Green, false);
  SetTowerLight(TowerLight::Red, false);
  SetTowerLight(TowerLight::Yellow, false);
}

void LineFollowerStartupTest::MotorsTest() {
  Bluetooth.println("Which motor? (0 left, 1 right, 2 both)");
  const int motor_number = ReadBluetoothInt(); 

  if (motor_number < 0 || motor_number > 2) {
    Bluetooth.println("Bad motor number, exiting");
    return;
  }

  Bluetooth.println("PWM duty cycle?");
  const float pwm_dc = ReadBluetoothFloat();

  if (abs(pwm_dc) > 1) {
    Bluetooth.println("Bad pwm dc, exiting");
    return;
  }

  char float_str[20];
  dtostrf(pwm_dc, 7, 4, float_str);
  char buf[50];
  sprintf(buf, "Motor %d, pwm %s, press '1' for OK", motor_number, float_str);
  Bluetooth.println(buf);

  while(!Bluetooth.available()) {
    delay(100);
  }
  char read = Bluetooth.read();

  if (read != '1') {
    Bluetooth.println("Exiting");
    return;
  }

  Bluetooth.println("Press any key to exit");

  EnableMotor(Motor::Left, true);
  EnableMotor(Motor::Right, true);
  SetPiston(PistonState::Down);

  switch(motor_number) {
    case 0:
      SetMotorPwm(Motor::Left, pwm_dc * 255.0);
      SetMotorPwm(Motor::Right, 0);
      break;
    case 1:
      SetMotorPwm(Motor::Left, 0);
      SetMotorPwm(Motor::Right, pwm_dc * 255.0);
      break;
    case 2:
      SetMotorPwm(Motor::Left, pwm_dc * 255.0);
      SetMotorPwm(Motor::Right, pwm_dc * 255.0);
      break;
  }

  BatteryMeter meter_;

  while(!Bluetooth.available()) {
    meter_.OnAnalogSample(ReadBatteryMeter());
    char float_str[20];
    dtostrf(meter_.Output_V(), 7, 4, float_str);
    char buf[50];
    sprintf(buf, "Battery: %s V", float_str);
    Bluetooth.println(buf);

    delay(100);
  }
  read = Bluetooth.read();

  EnableMotor(Motor::Left, false);
  EnableMotor(Motor::Right, false);
  SetMotorPwm(Motor::Left, 0);
  SetMotorPwm(Motor::Right, 0);
}

void LineFollowerStartupTest::BatteryMeterTest() {
  BatteryMeter meter_;
  
  while(!Bluetooth.available()) {
    meter_.OnAnalogSample(ReadBatteryMeter());

    static int loop_counter = 0;
    if (loop_counter++ > 500) {
      loop_counter = 0;
      char float_str[20];
      dtostrf(meter_.Output_V(), 7, 4, float_str);
      char buf[50];
      sprintf(buf, "Battery: %s V", float_str);
      Bluetooth.println(buf);
    }
   
    delay(1);
  }
  Bluetooth.read();
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
  Bluetooth.println("Battery meter test: Press 3");
  Bluetooth.println("Buttons test: Press 4");
  Bluetooth.println("Line follower test: Press 5");
  Bluetooth.println("Adjust PID Kp: Press 6");
  Bluetooth.println("Adjust PID Kd: Press 7");
  Bluetooth.println("Range sensor test: Press 8");
  Bluetooth.println("Motors test: Press 9");
  char option = TestPrompt("Choose a test");

  switch(option) {
    case '1': StartupTest(); break;
    case '2': LineSensorsTest(); break;
    case '3': BatteryMeterTest(); break;
    case '4': ButtonsTest(); break;
    case '5': LineFollowerTest(); break;
    case '6': AdjustPidKp(); break;
    case '7': AdjustPidKd(); break;
    case '8': RangeSensorTest(); break;
    case '9': MotorsTest(); break;
    default:
      Bluetooth.println("Invalid option");
      break;
  }
}

}  // namespace line_follower
