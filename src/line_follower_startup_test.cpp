#include "line_follower_startup_test.h"

#include <stdint.h>

#include <Arduino.h>
#include "hardware.h"
#include "util.h"
#include "line_sensor.h"
#include "current_sensor.h"
#include "button_debounce.h"
#include "line_follower.h"

namespace line_follower {

LineFollowerStartupTest::LineFollowerStartupTest() {}

#define Bluetooth Serial

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

  Bluetooth.read();
}

void LineFollowerStartupTest::LineSensorsTest() {
  Bluetooth.println("Line sensors test!");
  LineSensor line_sensor = {};

  TESTPROMPTRETURN("Piston down");
  SetPiston(PistonState::Down);
  delay(10000);
  SetPiston(PistonState::Idle);

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
  SetMotorPwm(Motor::Left, 200);
  SetMotorPwm(Motor::Right, 200);  

  while(!Bluetooth.available()) {
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
  Bluetooth.read();
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
  Bluetooth.println("Press Up button to transition up");
  Bluetooth.println("Press Down button to transition down");

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
      if (read == '\n') break;
      buf[buf_index++] = read;  
    }
  }

  return atof(buf);
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
  Bluetooth.println("Line follower test: Press 5");
  Bluetooth.println("Adjust PID Kp: Press 6");
  Bluetooth.println("Adjust PID Kd: Press 7");
  int option = TestPrompt("Choose a test") - (int)'0';

  switch(option) {
    case 1: StartupTest(); break;
    case 2: LineSensorsTest(); break;
    case 3: CurrentSensorsTest(); break;
    case 4: ButtonsTest(); break;
    case 5: LineFollowerTest(); break;
    case 6: AdjustPidKp(); break;
    case 7: AdjustPidKd(); break;
    default:
      Bluetooth.println("Invalid option");
      break;
  }
}

}  // namespace line_follower
