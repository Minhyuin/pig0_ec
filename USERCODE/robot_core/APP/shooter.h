#ifndef SHOOTER_H
#define SHOOTER_H

#include "drv_conf.h"
#include HAL_INCLUDE
#include "Base/rc.h"
#include "Devices/MOTOR/motor_headers.h"
#include "behaviour.h"

#define shooter_init() shooter17_init()
#define shooter_ctrl_loop(_rc_, _probot_) shooter17_ctrl_loop(_rc_, _probot_)

#define DEFAULT_SHOOT_SPEED 23.0f
#define MAX_SHOOT_SPEED 30.0f
#define D_FRIC 0.06f // m
#define C_FRIC (D_FRIC * PI)

#define DIAL_RR (5.f / 2.f)

#define STUCK_MAX_TIME_CNT 35
#define SINGLE_STUCK_MAX_TIME_CNT 10
#define STUCK_SPEED 5
#define STUCK_CURRENT 2.0f        // 安
#define STUCK_RESTORE_RPM_INV -80 // 卡弹恢复的反转速度
#define STUCK_RESTORE_RPM -20     // 卡弹恢复的正转速度
#define STUCK_REVERSE_TIME_CNT 500
#define STUCK_FORWARD -1
#define STUCK_REVERSE 1

#define S_CURVE_DIAL_ACC 0.5f

#define C_FRIC_WHEEL (0.06f * PI)
#define C_PUSHER_DIAL (0.08f * PI)

#define DIAL_RPM 135.f // 135 20Hz
#define DIAL_LOW_RPM 40.f

// 临桂县15℃ 6000为18m/s
// 临桂县15℃ 6800为23m/s
// 临桂县24℃ 6800为25m/s
// 临桂县24℃ 6300为24m/s
// 临桂县25℃ 6150为25m/s
// 临桂县25℃ 6000为23m/s
// 临桂县23℃ 6400为22m/s
#define FRIC_RPM 6000
// (DEFAULT_SHOOT_SPEED / C_FRIC * 60.f) // 不带减速箱的rpm
#define FRIC_TEST_RPM 2000.f

#define ONE_SHOOT_ANG 1.46911257f // 调参 使用abs_angle相减

#define MIN_SHOOT_HEAT 45
#define MIN_ONE_SHOOTocbe_HEAT 90

void shooter42_init(void);
HAL_StatusTypeDef shooter42_ctrl_loop(float dt, robot_t *robot);

void shooter17_init(void);
HAL_StatusTypeDef shooter17_ctrl_loop(float dt, robot_t *robot);

uint8_t get_single_shooting_flag(void);
uint8_t *get_heat_enable_flag_ptr(void);

#endif
