#ifndef _ROBOT_H
#define _ROBOT_H

#include "drv_conf.h"
#include "main.h"
#include "usbd_def.h"
#include CAN_INCLUDE
#include HAL_INCLUDE

#include "Base/bsp_dwt.h"
#include "Base/ext_imu.h"
#include "Base/power_ctrl.h"
#include "Base/rc.h"
#include "Base/referee.h"
#include "Base/super_cap.h"

#include "Devices/BMI088/drv_bmi088.h"
#include "Devices/MOTOR/motor_headers.h"

// #include "Devices/LIS3MDL/LIS3MDL.h"

#include "arm_math.h"

#include "Algorithm/filter.h"
#include "Algorithm/pid.h"
#include "Algorithm/pid_fuzzy.h"
#include "Algorithm/pid_ladrc.h"
#include "Algorithm/pid_leso.h"

#include "tools/asr.h"
#include "tools/buzzer.h"
#include "tools/flash.h"
#include "tools/led.h"
#include "tools/vofa.h"
#include "tools/ws2812.h"

#include "APP/behaviour.h"
#include "APP/boards_interact.h"
#include "APP/bus_detect.h"
#include "APP/chassis.h"
#include "APP/gimbal.h"
#include "APP/imu_bmi088.h"
#include "APP/shooter.h"
#include "Base/vision.h"

#include "OS/drv_freertos/include/self_delay.h"
#include "OS/robot_task.h"

#include "tests/test.h"

// typedef struct{
//     void (*init)(void);
//     void (*control)(void);
// }chassis_t;
// typedef struct{
//     void (*init)(void);
//     void (*control)(void);
// }gimbal_t;
// typedef struct{
//     void (*init)(void);
//     void (*control)(void);
// }shooter_t;

void robot_init(void);
robot_t *get_robot_ptr(void);

void diag_slow_time_cycle(void);
void robot_m_tim_cycle(void);
void robot_h_tim_cycle(void);
void robot_imu_tim_cycle(void);
void robot_main_loop(void);

void robot_state_update(void);
void robot_bus_detect(void);
void robot_gimbal_ctrl(void);
void robot_chassis_ctrl(void);
void robot_shooter_ctrl(void);
void robot_test_loop(void);

void robot_ui_ctrl(void);
void robot_ui_send(void);

#define ROBOT_DELAY(n) robot_delay_us_tim(n) // 选择开硬件定时器实现阻塞延时
#define ROBOT_DELAY_MS(n)                                                      \
  for (uint32_t i = 0; i < (n); i++)                                           \
    ROBOT_DELAY(1000); // 选择开硬件定时器实现阻塞延时
void robot_delay_us_tim(uint32_t period);
void robot_delay_us_systick(uint32_t period);
#endif //_ROBOT_TASK_H
