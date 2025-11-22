#include "can_task.hpp"

sp::CAN can1(&hcan1);

extern float trigger_given_torque;
extern sp::LK_Motor trigger_motor;

// 电机使能
extern bool motor_enable;
uint16_t motor_enable_freq = 0;
uint16_t can_send_freq = 0;

static void can_filter_init();
extern "C" void can_task(void * argument)
{
  can_filter_init();
  osDelay(500);  // 等待系统稳定

  while (1) {
    trigger_motor.cmd_torque(trigger_given_torque);
    trigger_motor.write_torque(can1.tx_data);
    can1.send(trigger_motor.tx_id);

    //can_send_freq = (can_send_freq + 1) % 100;
    osDelay(1);
  }
}

// 初始化CAN过滤器
static void can_filter_init()
{
  can1.config();
  can1.start();
}

// CAN接收回调函数
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == trigger_motor.rx_id) {
        trigger_motor.read_state1(can1.rx_data);
        trigger_motor.read_state2(can1.rx_data);
      }
    }
  }
}