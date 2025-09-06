/**
 * @file motor_conf_sample.h
 *
 *
 * @brief 电机库主要配置示例文件
 * @note 根据下面的修改指引 创建副本并改名为motor_conf.h于根目录
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#ifndef MOTOR_CONF_H_
#define MOTOR_CONF_H_

#include "./motor_def.h"
#include "drv_conf.h"
#include "motor_def.h"
#ifndef HAL_INCLUDE
#error "HAL_INCLUDE is not defined"
#else
#include HAL_INCLUDE // 本宏来自 drv_conf:stm32h7xx_hal.h
#endif
#ifndef CAN_INCLUDE
#error "CAN_INCLUDE is not defined"
#else
#include CAN_INCLUDE // 本宏来自 drv_conf:fdcan.h
#endif               // ! CAN_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONF_FILE "motor_conf.h"

#if 0
 __  __  ___ _____ ___  ____        ____ ___  _   _ _____ 
|  \/  |/ _ \_   _/ _ \|  _ \      / ___/ _ \| \ | |  ___|
| |\/| | | | || || | | | |_) |    | |  | | | |  \| | |_   
| |  | | |_| || || |_| |  _ <     | |__| |_| | |\  |  _|  
|_|  |_|\___/ |_| \___/|_| \_\     \____\___/|_| \_|_|    

 _____ ___  _     _     _____        _____ _   _  ____ 
|  ___/ _ \| |   | |   / _ \ \      / /_ _| \ | |/ ___|
| |_ | | | | |   | |  | | | \ \ /\ / / | ||  \| | |  _ 
|  _|| |_| | |___| |__| |_| |\ V  V /  | || |\  | |_| |
|_|   \___/|_____|_____\___/  \_/\_/  |___|_| \_|\____|

#endif
#if 1
#define WHEELS_NUM           4 // 轮毂电机数量

/*--------------------------------*/
#define GIMBAL_MOTORS_HCAN   hcan1
#define GIMBAL_MOTORS_NUM    2
#define GIMBAL_MOTORS_INDEX  1 - 1 // init_index:0
/*--------------------------------*/
#define SHOOTER_MOTORS_HCAN  hcan2
#define FRIC_MOTORS_HCAN     hcan2
#define DIAL_MOTORS_HCAN     hcan2
#define SHOOTER_MOTORS_NUM   3
#define SHOOTER_MOTORS_INDEX (GIMBAL_MOTORS_INDEX + GIMBAL_MOTORS_NUM) // init_index:2
#define FRIC_MOTORS_NUM      2
#define FRIC_MOTORS_INDEX    SHOOTER_MOTORS_INDEX // init_index:2
#define DIAL_MOTORS_NUM      1
#define DIAL_MOTORS_INDEX    (FRIC_MOTORS_INDEX + FRIC_MOTORS_NUM) // init_index:4
/*--------------------------------*/
#define CHASSIS_MOTORS_HCAN  hcan1
#define CHASSIS_MOTORS_NUM   4
#define CHASSIS_MOTORS_INDEX (SHOOTER_MOTORS_INDEX + SHOOTER_MOTORS_NUM) // init_index:5
#define WHEELS_MOTORS_INDEX  CHASSIS_MOTORS_INDEX
#define SPECIAL_MOTORS_INDEX (CHASSIS_MOTORS_INDEX + WHEELS_NUM)
/*--------------------------------*/
#define ARM_MOTORS_HCAN      hcan1
#define ARM_MOTORS_NUM       0
#define ARM_MOTORS_INDEX     (CHASSIS_MOTORS_INDEX + CHASSIS_MOTORS_NUM)
/*--------------------------------*/
#define TEST_MOTORS_HCAN     hcan1
#if USE_TEST_FILE
#define TEST_MOTORS_NUM 1
#else
#define TEST_MOTORS_NUM 0
#endif
#define TEST_MOTORS_INDEX    (ARM_MOTORS_INDEX + ARM_MOTORS_NUM) // init_index:9
/*--------------------------------*/
#define NUM_OF_RM_QUAD_MOTOR 7

#define NUM_OF_LK_MOTOR      0
#define NUM_OF_LK_QUAD_MOTOR 0

#define NUM_OF_DM_MOTOR      2
#define NUM_OF_DM_QUAD_MOTOR 0

#define NUM_OF_UT_MOTOR      0

#define NUM_OF_ROBOT_MOTOR                                                             \
    (NUM_OF_RM_QUAD_MOTOR + NUM_OF_LK_MOTOR + NUM_OF_LK_QUAD_MOTOR + NUM_OF_DM_MOTOR + \
     NUM_OF_DM_QUAD_MOTOR + NUM_OF_UT_MOTOR)

typedef union {
    struct {
        // INDEX 1-1
        motor_t pitch; // pitch电机
        motor_t yaw;   // yaw

        // INDEX 3-1
        motor_t R_fric; // 右摩擦轮
        motor_t L_fric; // 左摩擦轮
        motor_t dial;   // 拨弹盘

        // INDEX 6-1
        motor_t RF_wheel; // 右前轮
        motor_t LF_wheel; // 左前轮
        motor_t LB_wheel; // 左后轮
        motor_t RB_wheel; // 右后轮

#if USE_TEST_FILE
        motor_t test; // 测试用电机
#endif
    };
    motor_t _[NUM_OF_ROBOT_MOTOR]; // 方便遍历用
} motors_t;
#endif
#if 0
    _    ____   _____     _______ 
   / \  | __ ) / _ \ \   / / ____|
  / _ \ |  _ \| | | \ \ / /|  _|  
 / ___ \| |_) | |_| |\ V / | |___ 
/_/   \_\____/ \___/  \_/  |_____|

 __  __  ___ _____ ___  ____        ____ ___  _   _ _____ 
|  \/  |/ _ \_   _/ _ \|  _ \      / ___/ _ \| \ | |  ___|
| |\/| | | | || || | | | |_) |    | |  | | | |  \| | |_   
| |  | | |_| || || |_| |  _ <     | |__| |_| | |\  |  _|  
|_|  |_|\___/ |_| \___/|_| \_\     \____\___/|_| \_|_|
#endif

#if USE_TEST_FILE
#warning "Test Motor is enabled!"
#endif
#define NUM_OF_ALL_MOTOR \
    (GIMBAL_MOTORS_NUM + SHOOTER_MOTORS_NUM + CHASSIS_MOTORS_NUM + ARM_MOTORS_NUM + TEST_MOTORS_NUM)
#if NUM_OF_ALL_MOTOR != NUM_OF_ROBOT_MOTOR
#error "NUM_OF_ALL_MOTOR is not correct!"
// 此项报错 则说明motors_t里的电机个数与 (NUM_OF_RM_MOTOR + NUM_OF_LK_MOTOR +
// NUM_OF_DM_MOTOR) 不匹配
#endif

/*-----------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif
