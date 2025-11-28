#include "trigger_task.hpp"

const float DT_1000HZ = 0.001f;

sp::PID Trigger_Angle_Pid(DT_1000HZ, 60.0f, 0.01f, 0.0f, 2.0f, 0.6f, 0.25f, true, true);
sp::PID Trigger_Speed_Pid(DT_1000HZ, 0.5f, 1.5f, 0.0f, 2.4f, 0.6f, 0.25f, false, true);

sp::LK_Motor trigger_motor(1);

extern sp::DBus remote;
extern sp::Plotter plotter;
extern sp::CAN can1;

const int Shoot_rate = 5;                      //1Hz发射频率
const float Shoot_per_angle = sp::PI / 10.0f;  //每次发射增加的角度(弧度)

bool motor_enable = false;  // 电机使能标志
float trigger_given_torque = 0.0f;
float absolute_angle_set = trigger_motor.angle;
int last_remote_sw_l = int(sp::DBusSwitchMode::DOWN);

// speed正弦波测试参数
static float sine_wave_amplitude = 2.0f;  // 振幅（弧度/秒）
static float sine_wave_frequency = 1.0f;  // 频率（Hz）
float sine_wave_output = 0.0f;
int count = 0;
// 正弦波生成函数, 用于测试
static float generate_sine_wave_output()
{
  // 计算经过的时间（秒）
  float elapsed_time = (++count) / 1000.0f;

  // 生成正弦波：A * sin(2πft)
  sine_wave_output = sine_wave_amplitude * sinf(2.0f * M_PI * sine_wave_frequency * elapsed_time);

  return sine_wave_output;
}

extern "C" void trigger_task()
{
  osDelay(500);
  int time_counter = 0;

  while (1) {
    if (remote.sw_r == sp::DBusSwitchMode::MID) {
      motor_enable = true;
    }
    else {
      motor_enable = false;
      time_counter = 0;
    }

    if (motor_enable) {
      if (remote.sw_l == sp::DBusSwitchMode::UP) {
        time_counter++;
        if (time_counter % (1000 / Shoot_rate) == 0) {  //阶跃连发
          time_counter %= (1000 / Shoot_rate);
          absolute_angle_set = sp::limit_angle(absolute_angle_set + Shoot_per_angle);
        }
      }
      else if (
        remote.sw_l == sp::DBusSwitchMode::MID &&
        last_remote_sw_l != int(sp::DBusSwitchMode::MID)) {
        absolute_angle_set = sp::limit_angle(absolute_angle_set + Shoot_per_angle);  //单次发射
      }

      Trigger_Angle_Pid.calc(absolute_angle_set, trigger_motor.angle);

      Trigger_Speed_Pid.calc(Trigger_Angle_Pid.out, trigger_motor.speed);

      //Trigger_Speed_Pid.calc(2.0f + generate_sine_wave_output(), trigger_motor.speed);
      trigger_given_torque = Trigger_Speed_Pid.out;
      //trigger_given_torque = 0.1f;  //测试用常量扭矩
    }
    else {
      trigger_given_torque = 0.0f;
    }
    last_remote_sw_l = int(remote.sw_l);
    osDelay(1);
  }
}
