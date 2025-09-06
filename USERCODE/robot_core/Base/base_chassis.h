#ifndef _BASE_CHASSIS_H
#define _BASE_CHASSIS_H

#include "drv_conf.h"
#include HAL_INCLUDE

#include "APP/behaviour.h"
#include "Algorithm/imu_fusion.h"
#include "Algorithm/pid.h"
#include "Base/power_ctrl.h"
#include "Devices/MOTOR/motor_headers.h"

/**
 * @brief 四轮rpm
 */
typedef union {
  struct {
    float M_RF; ///< motor of the right front 0
    float M_LF; ///< motor of the left front 1
    float M_LB; ///< motor of the left back 2
    float M_RB; ///< motor of the right back 3
  };
  float _[WHEELS_NUM];
} general_wheels_state_t;

HAL_StatusTypeDef general_chassis_base_ctrl(robot_t *robot, motors_t *motors,
                                            chassis_power_lim_t *pow_lim,
                                            pid_struct_t wheels_pPID[4],
                                            pid_struct_t chassis_pPID[3]);

/////////////////////////////////////////////////////////////////////////////
/*轮的尺寸*/
// reference: https://www.robomaster.com/zh-CN/products/components/detail/135
#if CHASSIS_TYPE == MECANUM
#define D_WHEEL (0.1525f) ///< diameter of mecanum wheel in meter
#elif CHASSIS_TYPE == OMNI
#define D_WHEEL (0.1625f) ///< diameter of omni wheel in meter
#endif

#define C_WHEEL (PI * D_WHEEL) ///< circumference of wheel in meter
/*底盘机械参数定义，
两者分别为左或右前轮到旋转中心（云台Yaw轴）的俯视水平距离和垂直距离之和
和左或右后轮到旋转中心（云台Yaw轴）的俯视水平距离和垂直距离之和
*/
#define KXY_FRONT 0.344f ///< front wheels X+Y(distance,m)
#define KXY_BACK 0.344f  ///< back wheels X+Y(distance,m)
#define K_XY 0.352295f   ///< wheels X+Y(distance,m)
#define K_R 0.24911f
#define K_W 0.402f //<wheelbase of balanced infantry

#endif
