#include "trigger_task.hpp"

const float DT_1000HZ = 0.001f;

sp::PID Trigger_Angle_Pid(DT_1000HZ, 0.1f, 0.0f, 0.0f, 2.0f, 0.6f, 0.25f, false, true);
sp::PID Trigger_Speed_Pid(DT_1000HZ, 1.0f, 0.0f, 0.0f, 2.4f, 0.6f, 0.25f, false, true);

sp::LK_Motor trigger_motor(1);

extern sp::DBus remote;
extern sp::Plotter plotter;
extern sp::CAN can1;

const int Shoot_rate = 1;                      //1Hz发射频率
const float Shoot_per_angle = sp::PI / 10.0f;  //每次发射增加的角度(弧度)

bool motor_enable = false;  // 电机使能标志
float trigger_given_torque = 0.0f;
float absolute_angle_set = trigger_motor.angle;
extern "C" void trigger_task()
{
  osDelay(500);
  int time_counter = 0;

  while (1) {
    if (remote.sw_r == sp::DBusSwitchMode::MID && remote.sw_l == sp::DBusSwitchMode::UP) {
      motor_enable = true;
      trigger_motor.write_state1(can1.tx_data);
      can1.send(trigger_motor.tx_id);
    }
    else {
      motor_enable = false;
      time_counter = 0;
    }

    if (motor_enable) {
      time_counter++;
      if (time_counter % (1000 / Shoot_rate) == 0) {
        time_counter %= (1000 / Shoot_rate);
        absolute_angle_set = sp::limit_angle(absolute_angle_set + Shoot_per_angle);
      }
      Trigger_Angle_Pid.calc(absolute_angle_set, trigger_motor.angle);
      Trigger_Speed_Pid.calc(Trigger_Angle_Pid.out, trigger_motor.speed);
      trigger_given_torque = Trigger_Speed_Pid.out;
      trigger_given_torque = 0.1f;  //测试用常量扭矩
    }
    else {
      trigger_given_torque = 0.0f;
    }

    osDelay(1);
  }
}
