#include "io/plotter/plotter.hpp"
#include "trigger/trigger_task.hpp"
extern sp::DBus remote;
extern sp::LK_Motor trigger_motor;
extern float trigger_given_torque;
extern sp::PID Trigger_Angle_Pid;
extern sp::PID Trigger_Speed_Pid;
extern sp::CAN can1;
extern float absolute_angle_set;
extern float sine_wave_output;
extern float absolute_angle_add;
sp::Plotter plotter(&huart1);
extern "C" void plotter_task()
{
  while (1) {
    plotter.plot(
      float(remote.sw_r), float(remote.sw_l), trigger_motor.angle, trigger_motor.speed,
      trigger_motor.torque, absolute_angle_add,
      // trigger_given_torque, absolute_angle_set, Trigger_Angle_Pid.out,
      // Trigger_Speed_Pid.out,
      2.0f + sine_wave_output);

    osDelay(4);
  }
}