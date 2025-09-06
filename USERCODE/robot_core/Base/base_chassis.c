/**
 * @file base_chassis.c
 * @brief 底盘底层
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "base_chassis.h"

#include "Algorithm/filter.h"
#include "Algorithm/imu_fusion.h"
#include "Devices/MOTOR/motor_headers.h"

#include "APP/chassis.h"

#include "APP/behaviour.h"

#include <math.h>
#include <string.h>
/**
 * @note 麦轮及全向轮轮毂分布
 * 电机id及位置 注意要设置好相应电调或电机id
 *      前
 *	  2   1
 *  左      右
 *	  3   4
 *      后
 *
 * 若为十字型全向轮，则电机id及位置
 *       前
 *       2
 *  左3     1 右
 *       4
 *       后
 */
#define OMNI_X 0
#define OMNI_H 1
/*麦克纳姆轮运动学正向解算，即从四轮轮速换算到整车速度 轮速单位:rpm,
 * wz单位:rad/s*/
static void mecanum_forward(general_wheels_state_t *cur_wheel,
                            robot_state_t *cur_state) {
  cur_state->v_x =
      C_WHEEL *
      (cur_wheel->M_LF - cur_wheel->M_RF + cur_wheel->M_LB - cur_wheel->M_RB) /
      4.f / 60.f;
  cur_state->v_y = C_WHEEL *
                   (KXY_BACK * cur_wheel->M_RF + KXY_BACK * cur_wheel->M_LF -
                    KXY_FRONT * cur_wheel->M_LB - KXY_FRONT * cur_wheel->M_RB) /
                   (2.f * (KXY_BACK + KXY_FRONT)) / 60.f;
  cur_state->w_z =
      ((D_WHEEL / 2.f) *
       (cur_wheel->M_RF + cur_wheel->M_LF + cur_wheel->M_LB + cur_wheel->M_RB) /
       (2.f * (KXY_BACK + KXY_FRONT))) *
      (2.f * PI) / 60.f;
}
/*麦克纳姆轮运动学逆向解算，即从整车速度换算到四轮轮速 轮速单位:rpm,
 * wz单位:rad/s*/
static void mecanum_reverse(robot_state_t *exp_state,
                            general_wheels_state_t *exp_wheel) {
  exp_wheel->M_RF =
      (60.f / C_WHEEL) * (-exp_state->v_x + exp_state->v_y) +
      KXY_FRONT / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
  exp_wheel->M_LF =
      (60.f / C_WHEEL) * (exp_state->v_x + exp_state->v_y) +
      KXY_FRONT / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
  exp_wheel->M_LB =
      (60.f / C_WHEEL) * (exp_state->v_x - exp_state->v_y) +
      KXY_BACK / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
  exp_wheel->M_RB =
      (60.f / C_WHEEL) * (-exp_state->v_x - exp_state->v_y) +
      KXY_BACK / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
}
/*全向轮运动学逆向解算，即从整车速度换算到四轮轮速 轮速单位:rpm, wz单位:rad/s*/
static void omni_reverse(robot_state_t *exp_state,
                         general_wheels_state_t *exp_wheel, uint8_t OMNI_MODE) {
  if (OMNI_MODE == OMNI_X) {
    exp_wheel->M_RF =
        (60.f / C_WHEEL) * (-exp_state->v_x + exp_state->v_y) / sqrt(2) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
    exp_wheel->M_LF =
        (60.f / C_WHEEL) * (exp_state->v_x + exp_state->v_y) / sqrt(2) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
    exp_wheel->M_LB =
        (60.f / C_WHEEL) * (exp_state->v_x - exp_state->v_y) / sqrt(2) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
    exp_wheel->M_RB =
        (60.f / C_WHEEL) * (-exp_state->v_x - exp_state->v_y) / sqrt(2) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
  } else {
    exp_wheel->M_RF =
        (60.f / C_WHEEL) * (-exp_state->v_x) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
    exp_wheel->M_LF =
        (60.f / C_WHEEL) * (exp_state->v_y) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
    exp_wheel->M_LB =
        (60.f / C_WHEEL) * (exp_state->v_x) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
    exp_wheel->M_RB =
        (60.f / C_WHEEL) * (-exp_state->v_y) +
        K_XY / (D_WHEEL / 2.f) * exp_state->w_z * 60.f / (2.f * PI);
  }
}
/*全向轮运动学正向解算，即从四轮轮速换算到整车速度 轮速单位:rpm, wz单位:rad/s*/
static void omni_forward(general_wheels_state_t *cur_wheel,
                         robot_state_t *cur_state, uint8_t OMNI_MODE) {
  if (OMNI_MODE == OMNI_X) {
    cur_state->v_x = C_WHEEL *
                     (cur_wheel->M_LF - cur_wheel->M_RF + cur_wheel->M_LB -
                      cur_wheel->M_RB) *
                     sqrt(2) / 4.f / 60.f;
    cur_state->v_y = C_WHEEL *
                     (cur_wheel->M_LF + cur_wheel->M_RF - cur_wheel->M_LB -
                      cur_wheel->M_RB) *
                     sqrt(2) / 4.f / 60.f;
    cur_state->w_z = ((D_WHEEL / 2.f) *
                      (cur_wheel->M_RF + cur_wheel->M_LF + cur_wheel->M_LB +
                       cur_wheel->M_RB) /
                      4 / K_XY) *
                     (2.f * PI) / 60.f;

  } else {
    cur_state->v_x = C_WHEEL * (cur_wheel->M_RF - cur_wheel->M_LB) / 2.f / 60.f;
    cur_state->v_y = C_WHEEL * (cur_wheel->M_LF - cur_wheel->M_RB) / 2.f / 60.f;
    cur_state->w_z = ((D_WHEEL / 2.f) *
                      (cur_wheel->M_RF + cur_wheel->M_LF + cur_wheel->M_LB +
                       cur_wheel->M_RB) /
                      4.f / K_XY) *
                     (2.f * PI) / 60.f;
  }
}

static void chassis_clear_data(void) {
  // 主要用于清除一些积分
  ;
}

general_wheels_state_t cur_wheels;
general_wheels_state_t exp_wheels;

extern float max_vx_acc;
extern float max_vy_acc;

float M3508_real_torque_tx[4] = {0}; // motor_output 电机最终输出 编码值

/**
 * @brief 底盘解算函数
 * @param robot 机器人结构体指针
 * @param motors 电机集指针
 * @param pow_lim 电机功率限制结构体指针
 * @param wheels_pPID 轮子PID结构体指针
 * @param chassis_pPID 底盘PID结构体指针
 * @note ID一定要设置对 按照.h文件中的指示设置电机ID 不然就寄啦！
 * @return HAL_StatusTypeDef
 *          HAL_StatusTypeDef:
 *          @arg HAL_OK 进程正常
 *          @arg HAL_ERROR 机构电机掉线
 */
HAL_StatusTypeDef general_chassis_base_ctrl(robot_t *robot, motors_t *motors,
                                            chassis_power_lim_t *pow_lim,
                                            pid_struct_t wheels_pPID[4],
                                            pid_struct_t chassis_pPID[3]) {
  HAL_StatusTypeDef rslt = HAL_OK;
  robot_state_t *exp_state = robot->expected_state,
                *cur_state = robot->current_state;
  // 普通步兵一般不主动限制加速度 而一般由功率来限制

  /*中期考核时酌情主动限速↓*/
  LIMIT_MIN_MAX(exp_state->v_x, cur_state->v_x + max_vx_acc,
                cur_state->v_x - max_vx_acc);
  LIMIT_MIN_MAX(exp_state->v_y, cur_state->v_y + max_vy_acc,
                cur_state->v_y - max_vy_acc);
  // float max_wz_acc = 1.f;
  // LIMIT_MIN_MAX(exp_state->w_z, cur_state->w_z + max_wz_acc, cur_state->w_z -
  // max_wz_acc);
  /*中期考核时酌情主动限速↑*/
  for (uint8_t i = 0; i < WHEELS_NUM; i++)
    cur_wheels._[i] =
        iir_filter_3(motors->_[i + WHEELS_MOTORS_INDEX].real.rpm, i);
  // cur_wheels._[i] = motors->_[i+WHEELS_MOTORS_INDEX].real.rpm;

#if CHASSIS_TYPE == OMNI
  omni_forward(&cur_wheels, cur_state, OMNI_X);
  UNUSED(mecanum_forward);
#elif CHASSIS_TYPE == MECANUM
  mecanum_forward(&cur_wheels, cur_state);
  UNUSED(omni_forward);
#endif

  robot_state_t tmp_state;
  tmp_state.v_x = exp_state->v_x *
                  (1 + fabs(pid_calc_deadband(&chassis_pPID[0], exp_state->v_x,
                                              cur_state->v_x)));
  tmp_state.v_y = exp_state->v_y *
                  (1 + fabs(pid_calc_deadband(&chassis_pPID[1], exp_state->v_y,
                                              cur_state->v_y)));
  tmp_state.w_z =
      exp_state->w_z *
      (1 + fabs(pid_calc(&chassis_pPID[2], exp_state->w_z, cur_state->w_z)));
#if CHASSIS_TYPE == OMNI
  omni_reverse(&tmp_state, &exp_wheels, OMNI_X);
  UNUSED(mecanum_reverse);
#endif
#if CHASSIS_TYPE == MECANUM
  mecanum_reverse(&tmp_state, &exp_wheels);
  UNUSED(omni_reverse);
#endif

  float wheels_output[4] = {0};
  for (uint8_t i = 0; i < WHEELS_NUM; i++)
    wheels_output[i] =
        pid_calc(&wheels_pPID[i], exp_wheels._[i], cur_wheels._[i]);

  // 功率限制
  for (uint8_t i = 0; i < WHEELS_NUM;
       i++) { // 如果电机ID与.h规定的不一致会出问题喵
    pow_lim->raw_expt_Iq[i] =
        wheels_output[i] * C620_CURR_MAX / C620_CURR_DATA_MAX;
    pow_lim->real_omega[i] =
        rpm2radps(cur_wheels._[i]) * RR_of(motors->_[WHEELS_MOTORS_INDEX + i]);
    pow_lim->expt_omega[i] =
        rpm2radps(exp_wheels._[i]) * RR_of(motors->_[WHEELS_MOTORS_INDEX + i]);
    pow_lim->real_Iq[i] = motors->_[WHEELS_MOTORS_INDEX + i].real.current *
                          C620_CURR_MAX / C620_CURR_DATA_MAX;
  }
  chassis_power_distributor(pow_lim);
  if (is_motors_offline(CHASSIS_MOTORS)) {
    shutdown_motors_of_structure(CHASSIS_MOTORS);
    rslt |= HAL_ERROR;
  }
  if (robot->robot_control_flag) {
    for (uint8_t i = 0; i < WHEELS_NUM; i++) {
      M3508_real_torque_tx[i] = pow_lim->set_Iq[i] * Kn_M3508;
      // M3508_real_torque_tx[i] = 0; // debug
      LOAD_MOTOR_LPFTFF(motors->_[WHEELS_MOTORS_INDEX + i],
                        M3508_real_torque_tx[i]);
      motors->_[WHEELS_MOTORS_INDEX + i].AUX.EN = true;
    }
// ENABLE_ALL_DM_MOTOR(*motors);
// ENABLE_ALL_LK_MOTOR(*motors);
#if OS_ENABLE
    extern xSemaphoreHandle xMutex_set_all_motor_output;
    if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE) {
      rslt |= set_all_motor_output(CHASSIS_MOTORS);
      xSemaphoreGive(xMutex_set_all_motor_output);
    }
#else
    rslt |= set_all_motor_output(CHASSIS_MOTORS);
#endif
  } else {
    // DISABLE_ALL_DM_MOTOR(*motors);
    // DISABLE_ALL_LK_MOTOR(*motors);
    for (uint8_t i = 0; i < WHEELS_NUM; i++)
      LOAD_MOTOR_TFF(motors->_[WHEELS_MOTORS_INDEX + i], 0),
          motors->_[WHEELS_MOTORS_INDEX + i].AUX.EN = false;
    chassis_clear_data();
    rslt |= HAL_OK;
  }
  return rslt;
}
