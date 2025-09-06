/**
 * @file robot.c
 * @brief 机器人进程函数
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "robot.h"
#include "APP/ui_ctrl.h"
#include "main.h"
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

/*机器人变量*/
robot_t robot = {0};
robot_state_t expected_state = {0}, current_state = {0};

imu_data_t gimbal_imu_data = {0};
imu_data_t chassis_imu_data = {0};

rc_ctrl_t *rc_recv = NULL;

uint32_t now_timestamp = 0;

int8_t IMU_rslt = BMI08X_OK;
HAL_StatusTypeDef UART_rslt = HAL_OK;
uint16_t imu_init_cnt = 0;

/*计时变量*/
uint32_t detect_dwt_cnt = 0;
float detect_dt = 1.f / DETECT_TIM_FREQ;
uint32_t state_dwt_cnt = 0;
float state_dt = 1.f / STATE_TIM_FREQ;
uint32_t gimbal_dwt_cnt = 0;
float gimbal_dt = 1.f / GIMBAL_TIM_FREQ;
uint32_t chassis_dwt_cnt = 0;
float chassis_dt = 1.f / CHASSIS_TIM_FREQ;
uint32_t shooter_dwt_cnt = 0;
float shooter_dt = 1.f / SHOOTER_TIM_FREQ;
uint32_t test_dwt_cnt = 0;
float test_dt = 1.f / TEST_TIM_FREQ;

static int8_t imu_init(void) {
  int8_t rslt = 0;
  ROBOT_DELAY(50000); // 50ms for POR/BOR startup
  do {
    ROBOT_DELAY(3000); // 3ms startup
    rslt = init_bmi08x();
    if (rslt != BMI08X_OK) {
      imu_init_cnt++;
      // beep(5, 10);
      ROBOT_DELAY(5000);
    }
    if (imu_init_cnt > 5) {
      // imu init faile, reset all
      __set_FAULTMASK(1);
      NVIC_SystemReset();
      while (1)
        ;
    }
  } while (rslt != BMI08X_OK);
  gimbal_imu_data.raw_data.a_fullscale = ACCEL_RANGE_G * 9.7833f;
  gimbal_imu_data.raw_data.g_fullscale = GYRO_RANGE_DPS;
  chassis_imu_data.raw_data.a_fullscale = ACCEL_RANGE_G * 9.7833f;
  chassis_imu_data.raw_data.g_fullscale = GYRO_RANGE_DPS;
  enable_bmi08x_interrupt();
  return rslt;
}

static void beep_start_show(void) {
  beep(1, 1, 600);
  for (uint8_t i = 0; i < 3; i++) {
    buzzer_proc();
    ROBOT_DELAY_MS(100);
  }
  ROBOT_DELAY_MS(100);
  beep(1, 2, 400);
  for (uint8_t i = 0; i < 4; i++) {
    buzzer_proc();
    ROBOT_DELAY_MS(100);
  }
  ROBOT_DELAY_MS(100);
}

/**
 * @brief 机器人上电初始化
 * @note 初始框架配置 不初始stm32化外设相关
 * @retval None
 */
void robot_init(void) {
  DWT_Init(CORE_FREQ);
  __disable_irq();

  __HAL_RCC_CLEAR_RESET_FLAGS();

  UART_rslt |= rc_recv_dma_init();
#if USE_VT_RC_UART
  UART_rslt |= vt_rc_recv_dma_init();
#endif
  referee_recv_dma_init();
  // extern CAN_HandleTypeDef hcan1;
  // extern CAN_HandleTypeDef hcan2;
  // old_can_user_init(&hcan1);
  // old_can_user_init(&hcan2);
  // magazine_switch_init();
  behaviour_init(&robot);
#if !USE_TEST_FILE
#if (BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE) || !BOARDS_MODE
  chassis_init();
#endif

#if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
  shooter_init();
  gimbal_init();
#endif
#else
  motor_test_init();
#endif

#if BOARDS_MODE
  board_interact_init();
#endif

  ws2812_init();
  asr_cmd_init();

  IMU_rslt |= imu_init();
  // LIS3MDL_Init(LIS3MDL_Scale_16G,LIS3MDL_MODE_ULTRAHIGH,LIS3MDL_ODR_FAST);

  UI_Init();
	
	beep_start_show();

  __enable_irq();

}

/**
 * @brief 机器人自检函数 另外还负责给电机发0断控
 * @note 主要检测电机 CAN IMU RC等
 * @note 供给定时器使用
 * @retval None
 */
void diag_slow_time_cycle(void) {
  robot_bus_detect();
#if defined(STM32H723xx)
  RGB_loop(10, 0.5, 1);
#elif defined(STM32F407xx)
  RGB_loop(10, 0.5);
#endif
  buzzer_proc();
}

/**
 * @brief 机器人中速进程 200Hz
 * @note 供给定时器使用
 * @retval None
 */
void robot_m_tim_cycle(void) {
  rc_recv = get_rc_data_ptr();
  behaviour_ctrl_loop(rc_recv, &robot);

  robot_test_loop();

  robot_shooter_ctrl();

  robot_chassis_ctrl();

  robot_ui_ctrl();

  robot_ui_send();

  update_rc_last_key(robot.rc_sc);
}

/**
 * @brief 机器人高速进程 500Hz
 * @note 供给定时器使用
 * @retval None
 */
void robot_h_tim_cycle(void) {
  rc_recv = get_rc_data_ptr();
  behaviour_ctrl_loop(rc_recv, &robot);

  robot_gimbal_ctrl();

  update_rc_last_key(robot.rc_sc);
#if BOARDS_MODE
  board_interact_send();
#endif
}

/**
 * @brief 机器人bmi08x特供进程 1000Hz以下
 * @retval None
 */
void robot_imu_tim_cycle(void) {
#if MACHINE_TYPE == GIMBAL_MASTER
  imu_data_t *imu_data = &gimbal_imu_data;
#else
  imu_data_t *imu_data = &chassis_imu_data;
#endif
  IMU_rslt = get_imu_att(&imu_data->raw_data, &imu_data->fp_data,
                         &imu_data->eular, &imu_data->temperature);
  if (IMU_rslt == BMI08X_OK) {
    send_vision_request(imu_data->eular.roll * (2.0f * PI / IMU_RANGE),
                        imu_data->eular.pitch * (2.0f * PI / IMU_RANGE),
                        imu_data->eular.yaw * (2.0f * PI / IMU_RANGE));
    // float fbuf[3] = {imu_data->eular.roll * (2.0f * PI / IMU_RANGE),
    //                  imu_data->eular.pitch * (2.0f * PI / IMU_RANGE),
    //                  imu_data->eular.yaw * (2.0f * PI / IMU_RANGE)};
    // update_vofa_with_usb(fbuf, 3);
  }
  // 视觉要用
  now_timestamp += 1000.f / (uint32_t)IMU_TIM_FREQ; // 单位ms
}

/**
 * @brief 状态机
 */
void robot_state_update(void) {
  state_dt = DWT_GetDeltaT(&state_dwt_cnt);
  if (!state_dt)
    state_dt = 1.f / STATE_TIM_FREQ;
#if OS_ENABLE
  extern volatile xSemaphoreHandle xMutex_rc_update;
  if (xSemaphoreTake(xMutex_rc_update, portMAX_DELAY) == pdTRUE) {
    rc_recv = get_rc_data_ptr();
    xSemaphoreGive(xMutex_rc_update);
  }
#else
  rc_recv = get_rc_data_ptr();
#endif

  behaviour_ctrl_loop(rc_recv, &robot);

#if OS_ENABLE
  if (xSemaphoreTake(xMutex_rc_update, portMAX_DELAY) == pdTRUE) {
    update_rc_last_key(robot.rc_sc);
    xSemaphoreGive(xMutex_rc_update);
  }
#else
  update_rc_last_key(robot.rc_sc);
#endif
}
/**
 * @brief 异常检测
 */
void robot_bus_detect(void) {
  detect_dt = DWT_GetDeltaT(&detect_dwt_cnt);
  if (!detect_dt)
    detect_dt = 1.f / DETECT_TIM_FREQ;

  motor_switch(robot.robot_control_flag);
  magazine_ctrl_loop(robot.magazine_switch_flag);
  detect_loop(detect_dt);
}
/**
 * @brief 机器人云台控制
 */
void robot_gimbal_ctrl(void) {
  gimbal_dt = DWT_GetDeltaT(&gimbal_dwt_cnt);
  if (!gimbal_dt)
    gimbal_dt = 1.f / GIMBAL_TIM_FREQ;

#if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
#if !USE_TEST_FILE
  gimbal_ctrl_loop(gimbal_dt, &robot);
#endif
#endif
}
/**
 * @brief 机器人底盘控制
 */
void robot_chassis_ctrl(void) {
  chassis_dt = DWT_GetDeltaT(&chassis_dwt_cnt);
  if (!chassis_dt)
    chassis_dt = 1.f / CHASSIS_TIM_FREQ;
#if !USE_TEST_FILE
  chassis_ctrl_loop(chassis_dt, &robot);
#endif
}
/**
 * @brief 机器人射击控制
 */
void robot_shooter_ctrl(void) {
  shooter_dt = DWT_GetDeltaT(&shooter_dwt_cnt);
  if (!shooter_dt)
    shooter_dt = 1.f / SHOOTER_TIM_FREQ;

#if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
#if !USE_TEST_FILE
  shooter_ctrl_loop(shooter_dt, &robot);
#endif
#endif
}

/**
 * @brief 机器人UI控制
 */
void robot_ui_ctrl(void) {
  static uint32_t ui_ctrl_last_tick = 0;
  IntervalExecute(UI_Update, 1000 / UI_TIM_FREQ, &ui_ctrl_last_tick, HAL_GetTick);
}

/**
 * @brief 机器人UI调度发送任务
 */
void robot_ui_send(void) {
  static uint32_t ui_send_last_tick = 0;
  IntervalExecute(UI_Send, 1000 / UI_SEND_FREQ, &ui_send_last_tick, HAL_GetTick);
}

/**
 * @brief 机器人无指定时序进程 ??Hz
 * @retval None
 */
void robot_main_loop(void) {
  // 需要控制运行速度可以在这里加丐版延时
  // uint32_t tick_start = HAL_GetTick();
  // while (HAL_GetTick() - tick_start < 1000 / (uint32_t)MIN_MAIN_LOOP_FREQ);
  // update_vofa();
}
/**
 * @brief 机器人测试
 * @retval None
 */
void robot_test_loop(void) {
  test_dt = DWT_GetDeltaT(&test_dwt_cnt);
  if (!test_dt)
    test_dt = 1.f / TEST_TIM_FREQ;

  motor_test_loop(test_dt, &robot);
  // extern UART_HandleTypeDef huart1;
  //  HAL_GPIO_WritePin(RS485_TR_GPIO_Port,RS485_TR_Pin,GPIO_PIN_SET);
  //	HAL_UART_Transmit(&huart1,"U3TOU2",sizeof("U3TOU2")-1,0xFFFF);
  // HAL_GPIO_WritePin(RS485_TR_GPIO_Port,RS485_TR_Pin,GPIO_PIN_RESET);
}

extern TIM_HandleTypeDef DELAY_TIM_HANDLE;
void robot_delay_us_tim(uint32_t period) {
  __HAL_TIM_SetCounter(&DELAY_TIM_HANDLE, 0);
  __HAL_TIM_ENABLE(&DELAY_TIM_HANDLE);

  while ((__HAL_TIM_GetCounter(&DELAY_TIM_HANDLE)) < period)
    ;

  __HAL_TIM_DISABLE(&DELAY_TIM_HANDLE);
}
void robot_delay_us_systick(uint32_t period) {

  uint32_t tick_start, tick_now, reload;
  uint32_t tick_cnt = 0;
  reload = SysTick->LOAD;
  period = period * CORE_FREQ;
  tick_start = SysTick->VAL;
  while (1) {
    tick_now = SysTick->VAL;
    if (tick_now != tick_start) {
      if (tick_now < tick_start) {
        tick_cnt += tick_start - tick_now;
      } else {
        tick_cnt += reload - tick_now + tick_start;
      }
      tick_start = tick_now;
      if (tick_cnt >= period) {
        break;
      }
    }
  }
}
