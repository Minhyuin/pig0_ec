#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "../Base/rc.h"
#include "robot.h"

/**
 * @note RR表示机械结构减速比 eg:同步带2:1传动 yaw电机转2圈yaw轴转一圈 RR=2
 * !注意!本框架未完全适配PITCH_RR>1的情况
 * @note PITCH_BAL_POS 表示机械结构平衡位置 弧度制(给前馈写的 设0暂不启用)
 * @note PITCH_REL_RAD_MAX/MIN 真缓启动结束后 读机械极限角度时pitch_rad
 * 低头角度为MAX 抬头角度为MIN
 * @note GIMBAL_PITCH/YAW_MOTOR_CENTER 机器人摆正时的响应电机编码值
 */
#define PITCH_RR 1.f
#define YAW_RR 1.f

#define PITCH_REL_RAD_MAX 0.405739069f
#define PITCH_REL_RAD_MIN -0.411874533f

#define GIMBAL_PITCH_MOTOR_CENTER 3197
#define GIMBAL_YAW_MOTOR_CENTER 3048

#define PITCH_RES_CPST 0.0606796f

#define PITCH_BAL_POS 0.f

#define G_SOFT_START_SPEED_MOTOR 7       // 编码值
#define G_SOFT_START_SPEED_RAD 0.005366f // rad

extern float pitch_rad_center;
extern float yaw_rad_center;

extern float chassis_follow_yaw;

extern uint8_t is_gimbal_confirm_0degree;

#define SPINNING_GIMBAL_REVISE_K 0.25f

// RAD值通过MOTOR除以编码值映射到弧度得到
#define G_SOFT_START_DEADBAND_MOTOR 80
#define G_SOFT_START_DEADBAND_RAD 0.061996f
#define G_SOFT_START_MAX_CNT 2000

#define PATROL_YAW_SPEED 220
#define PATROL_PITCH_SPEED 100

/*底盘自旋校准因子，用于校正直接底盘逆向解算出的转速的比例误差
该参数直接影响无IMU的小陀螺时云台的漂移大小*/
#define K_CHASSIS_CAL 0.9514f

typedef enum {
  G_UNRESET = 0u,
  G_RESETING,
  G_RESETED,
} gimbal_reseting_state_t;
typedef enum { USE_IMU = 0u, USE_MOTOR } gimbal_ctrl_type_t;

void reset_chassis_angle(void);
uint8_t is_gimbal_reseted(void);
uint8_t *get_gimbal_reset_flag_ptr(void);
gimbal_ctrl_type_t *get_gimbal_ctrl_type_ptr(void);

void gimbal_init(void);
void gimbal_ctrl_loop(float dt, robot_t *robot);

#endif
