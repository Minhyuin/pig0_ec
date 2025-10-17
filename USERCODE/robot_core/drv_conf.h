/**
 * @file drv_conf.h
 *
 * @brief 机器人主要配置
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef _DRV_CONF_H
#define _DRV_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_INCLUDE "stm32f4xx_hal.h" ///< define HAL header
#define CAN_INCLUDE "Base/drv_can.h"  ///< define CAN header
#include "main.h"
#include "stdio.h"
/*-----------------------------------------------------------------*/
#ifndef RAM_PERSIST
#define RAM_PERSIST ///< 不用 RAM_PERSIST 以后研究研究可能用上
// #define RAM_PERSIST __attribute__((section("NO_INIT"),zero_init))  ///< marco
// to avoid variable initialized at reset

#endif

#include "math.h" ///< math库
#include "stdarg.h"
#include "stdbool.h" ///< bool类型定义
#include "stdint.h"  ///< int类型定义
#include "stdio.h"
#include "stdlib.h" ///< stdlib库
#include "string.h" ///< string库

#define _DEBUG 1 ///< enable debug function

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/*-----------------------------------------------------------------*/
///< OS_ENABLE 置1时使用操作系统
#define OS_ENABLE 0

#define CORE_FREQ 168         // Mhz
#define MAIN_FREQ (10.f)      // 要用test_point结合示波器测出
#define TIM3_IT_FREQ (10.f)   // 10hz(bus_detect定时器)
#define TIM9_IT_FREQ (1250.f) // 1250Hz IMU特供
#define TIM6_IT_FREQ (500.f)  // 200hz(中速定时器)
#define TIM7_IT_FREQ (1000.f) // 1000hz(高速定时器)

#define TIM1_IT_FREQ (25.f)
#define TIM4_IT_FREQ (400.f)
#define TIM8_IT_FREQ (400000.f)

#define M_TIM_FREQ TIM6_IT_FREQ
#define H_TIM_FREQ TIM7_IT_FREQ
#define IMU_TIM_FREQ TIM9_IT_FREQ
#define DETECT_TIM_FREQ TIM3_IT_FREQ

#define TEST_TIM_FREQ M_TIM_FREQ
#define STATE_TIM_FREQ H_TIM_FREQ

#define UI_TIM_FREQ 50.f
#define UI_SEND_FREQ (25.f) // 裁判系统UI链路接收上限为30Hz

#define M_TIM_HANDLE htim6
#define H_TIM_HANDLE htim7
#define IMU_TIM_HANDLE htim9
#define DETECT_TIM_HANDLE htim3

// 注意DELAY_TIM_HANDLE挂载在哪个APB上
#define DELAY_TIM_HANDLE htim5
#define DELAY_TIM_MAX_FREQ 84000000.f

#define BEEP_TIM_FREQ TIM4_IT_FREQ
#define MAGAZINE_TIM_FREQ TIM1_IT_FREQ
#define WS2812_TIM_FREQ TIM8_IT_FREQ

#define BEEP_TIM_HANDLE htim4
#define BEEP_TIM_CHANNAL TIM_CHANNEL_3

#define MAGAZINE_TIM_HANDLE htim1
#define MAGAZINE_TIM_CHANNAL TIM_CHANNEL_2

#define WS2812_TIM_HANDLE htim8
#define WS2812_TIM_CHANNAL TIM_CHANNEL_1

// 需要在CubeMX中定义TEST_PIN_GPIO_Port和TEST_PIN_Pin
#define Test0_Point_Start()                                                    \
  HAL_GPIO_TogglePin(TEST_PIN0_GPIO_Port, TEST_PIN0_Pin)
#define Test0_Point_Stop() Test0_Point_Start()
#define Test0_Point() Test0_Point_Start(), Test0_Point_Start()

#define Test1_Point_Start()                                                    \
  HAL_GPIO_TogglePin(TEST_PIN1_GPIO_Port, TEST_PIN1_Pin)
#define Test1_Point_Stop() Test1_Point_Start()
#define Test1_Point() Test1_Point_Start(), Test1_Point_Start()

/*-----------------------------noCmake-----------------------------*/
#if 1
#define USE_RC_UART 1
#define USE_VT_RC_UART 1
#define USE_REFEREE_UART 1
#define USE_EXT_CAN_IMU 1
#define USE_EXT_IMU_UART 0
#define USE_SUPER_CAP 1
#endif
/*-----------------------------------------------------------------*/
// 在C板中  图传串口和DEBUG串口 两者占用同一个串口链路 不能同时开
#define DEBUG_UART USART1
#define DEBUG_UART_HANDLE huart1

#define USE_LOG_UART 0
#ifdef USE_LOG_UART
#define LOG_UART DEBUG_UART
#define LOG_UART_HANDLE DEBUG_UART_HANDLE
#endif

#define USE_VOFA_UART 0
#ifdef USE_VOFA_UART
#define VOFA_UART DEBUG_UART
#define VOFA_UART_HANDLE DEBUG_UART_HANDLE
#endif

#if USE_VT_RC_UART
#define VT_RC_UART USART1
#define VT_RC_UART_HANDLE huart1
#endif

#if USE_RC_UART
#define RC_UART USART3
#define RC_UART_HANDLE huart3
#endif

// 注意在下板中使用的是哪个口
#if USE_REFEREE_UART
#define REFEREE_UART USART6
#define REFEREE_UART_HANDLE huart6
#define REFEREE_ASYNC_SEND 1
#endif

#if USE_SUPER_CAP
#define SUPER_CAP_CAN_HANDLE hcan1
#define CAP_CAN_FILTER_NUM 12
#endif

#if USE_EXT_CAN_IMU
#define IMU_USE_CAN CAN2
#define IMU_USE_CAN_HANDLE hcan2
#endif

#if USE_EXT_IMU_UART
#define EXT_IMU_UART UART1
#define EXT_IMU_UART_HANDLE huart1
#endif
/*-----------------------------------------------------------------*/
#define C2G_INTERACT_TIM_FREQ M_TIM_FREQ
#define G2C_INTERACT_TIM_FREQ H_TIM_FREQ

#define BOARDS_MODE 0 ///< 是否存在于多板系统中

#define CHASSIS_SLAVE 1
#define GIMBAL_MASTER 0

#if BOARDS_MODE

#define MACHINE_TYPE GIMBAL_MASTER ///< 选择上下板

#if MACHINE_TYPE == GIMBAL_MASTER
#define BOARDS_INTERACT_CAN CAN1
#define BOARDS_INTERACT_CAN_HANDLE hcan1
#define INTERACT_TIM_FREQ G2C_INTERACT_TIM_FREQ
#else
#define BOARDS_INTERACT_CAN CAN2
#define BOARDS_INTERACT_CAN_HANDLE hcan2
#define INTERACT_TIM_FREQ C2G_INTERACT_TIM_FREQ
#endif
#else

#define MACHINE_TYPE GIMBAL_MASTER ///< 单板控制下默认放在头部

#endif

#define G2C_CAN_BASE_ID 0x020
#define C2G_CAN_BASE_ID 0x030
/*-----------------------------------------------------------------*/
#define MOTOR_CAN_ENABLE 1
#if MOTOR_CAN_ENABLE == 1
/*CAN等待发送完成超时，单位毫秒，设为0即无阻塞的发送*/
#define CAN_TX_TIMEOUT 0 ///< timeout in ms, set to 0 to disable
#define CAN_DATA_LEN 8   ///< can frame length
///< MIN_MOTOR_REFRESH_FREQ为电机反馈数据刷新频率最低值,单位Hz,过低则会判断为离线

// 提醒 在motor.h中有参数需要人工设置

#define CHASSIS_TIM_FREQ M_TIM_FREQ
#define GIMBAL_TIM_FREQ H_TIM_FREQ // 云台需要更高精度  默认放置在高速loop中
#define SHOOTER_TIM_FREQ M_TIM_FREQ
#define MECANUM 1
#define OMNI 2
#define WHEELLEGGED 3
#define SWERVE 4

#define CHASSIS_TYPE OMNI

#define GENERAL_NUM_OF_WHEEL 4
#define BALANCED_NUM_OF_WHEEL 2
/**
 * @note  对于不同的底盘,需要做的适配就在base_drv/base_chassis.c中编写,
 */

#endif
/*-----------------------------------------------------------------*/
#define USE_REFEREE 0       ///< 是否使用裁判系统
#define READY_FOR_BATTLE 0  ///< 为0不会触发阵亡检测
#define USE_DT7_WHEEL 1     ///< 如果DT7遥控器拨轮坏了就置零不启用
#define USE_TEST_FILE 0     ///< 测试功能启用文件test.c
#define USE_GIMBAL_RESET 1  ///< 使能云台缓启动
#define USE_CHASSIS_RESET 1 ///< 使能底盘缓启动
#define USE_ROBOT_RESETING_SHOOTER_PROTECTION 1 ///< 机器人复位时禁用射击保护
#define USE_CENTER_FIRE 0                       ///< 使能中心火控
#define USE_CHASSIS_IMU 0                       ///< 使能底盘IMU
#define SPIN_DEBUG 1 ///< 给视觉调试的时候置1得到匀速小陀螺
#define USE_ASR 0    ///< 是否使用ASR播报功能(需要接入喇叭)
#define USE_WS2812 1 ///< 是否使用WS2812显示状态
/*-----------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
