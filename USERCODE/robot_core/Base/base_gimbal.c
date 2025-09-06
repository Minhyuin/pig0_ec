/**
 * @file base_gimbal.c
 *
 * @brief 云台底层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "base_gimbal.h"

#include "Base/ext_imu.h"
#include "Devices/MOTOR/motor_headers.h"

#include "base_chassis.h"

extern gimbal_ctrl_type_t gimbal_ctrl_type;

float pitch_exp_p = 0, yaw_exp_p = 0;
float pitch_exp_tff = 0, yaw_exp_tff = 0;
float vision_yaw_tff = 0, vision_pitch_tff = 0;
float vision_yaw_p = 0, vision_pitch_p = 0;

float delta_ang_pitch = 0, delta_ang_yaw = 0;
float yaw_torque_revise = 106.f;

// pitch前馈电压 单位V
float pitch_feedback = 0;
// pitch力平衡时的角度 单位编码值
int16_t pitch_balance_pos =
    PITCH_BAL_POS; // TODO:对于头部重量会变化的头 建议搞成自适应的
// 云台重力对应的电压
float gimbal_weight = GIMBAL_WEIGHT;

float gimbal_vel_max = 10.f;

/**
 * @brief 云台IMU系控制
 * @note 此版本由两个GM6020控制 搭配线性扩张观测器使用 (也可以不用)
 * @note 之后可以优化leso库
 * @attention 本框架使用的是电压控制GM6020
 */
HAL_StatusTypeDef
gimbal_imu_ctrl(motors_t *motors, robot_t *robot, pid_struct_t pitch_pid[2],
                pid_struct_t yaw_pid[2], leso_para_t *pitch_imu_leso,
                leso_para_t *yaw_imu_leso, one_vec_kf_t *kalman_pitch_filter,
                one_vec_kf_t *kalman_yaw_filter) {
  HAL_StatusTypeDef rslt = HAL_OK;
  motor_data_t *pitch_motor_pdata = &motors->pitch.real;
  motor_data_t *yaw_motor_pdata = &motors->yaw.real;
  robot_state_t *exp_state = robot->expected_state;
  robot_state_t *cur_state = robot->current_state;
  imu_data_t *imu = robot->gimbal_imu_data;

  if (imu == NULL) {
    return HAL_ERROR;
  }

  delta_ang_pitch =
      get_minor_arc(exp_state->pitch_ang, cur_state->pitch_ang, IMU_RANGE);
  delta_ang_yaw =
      get_minor_arc(exp_state->yaw_ang, cur_state->yaw_ang, IMU_RANGE);

  // 以下是通过双环pid输出期望电压给电机 控制角度和转速
  // TODO:2023年10月GM6020出电流控制模式了 建议通过电流来统一控制扭矩

  kalman1_filter(kalman_pitch_filter,
                 pitch_motor_pdata->current +
                     pitch_motor_pdata->rpm / Kn_RPM_GM6020 *
                         (GM6020_VOLT_DATA_MAX / GM6020_VOLT_RATED));
  kalman1_filter(kalman_yaw_filter,
                 yaw_motor_pdata->current +
                     yaw_motor_pdata->rpm / Kn_RPM_GM6020 *
                         (GM6020_VOLT_DATA_MAX / GM6020_VOLT_RATED));

  float pitch_imu_rpm = imu->fp_data.gy / (2.f * PI) * 60.0f;
  float yaw_imu_rpm = imu->fp_data.gz / (2.f * PI) * 60.0f;

  // 调前馈的时候 将pid_imu的输出给0
  pitch_feedback =
      sinf(motors->pitch.real.abs_angle / PITCH_RR - pitch_balance_pos) *
      gimbal_weight;

  pitch_exp_tff =
      fir_filter_5(CLAMP(pid_deadband_leso_dualloop(
                             pitch_pid, pitch_imu_leso, delta_ang_pitch,
                             pitch_imu_rpm, kalman_pitch_filter->x) +
                             pitch_feedback,
                         GM6020_VOLT_MAX),
                   GIMBAL_PITCH_FIR_CH);
  yaw_exp_tff = fir_filter_5(
      CLAMP(pid_deadband_leso_dualloop(yaw_pid, yaw_imu_leso, delta_ang_yaw,
                                       yaw_imu_rpm, kalman_yaw_filter->x),
            GM6020_VOLT_MAX),
      GIMBAL_YAW_FIR_CH);

  pitch_exp_p = motors->pitch.AUX.xout.dat + pitch_motor_pdata->abs_angle +
                delta_ang_pitch / IMU_RANGE * (2 * PI);
  yaw_exp_p = motors->yaw.AUX.xout.dat + yaw_motor_pdata->abs_angle +
              delta_ang_yaw / IMU_RANGE * (2 * PI);

// yaw扭力补偿 没测不敢用
#if 0
    if(robot->chassis_tag == C_SPIN_MODE)
    yaw_exp_tff += cur_state->w_z * yaw_torque_revise;
    LIMIT_MIN_MAX(yaw_exp_tff,-GM6020_VOLT_DATA_MAX,GM6020_VOLT_DATA_MAX);
#endif

  // pitch_exp_tff = 0;   // debug
  // yaw_exp_tff = 0;     // debug
  if (is_motors_offline(GIMBAL_MOTORS)) {
    shutdown_motors_of_structure(GIMBAL_MOTORS);
    rslt |= HAL_ERROR;
  }
  if (robot->robot_control_flag) {
    if (motors->pitch.work_mode == MIT_TT ||
        motors->pitch.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->pitch, pitch_exp_tff);
    else {
      LOAD_MOTOR_PDES(motors->pitch, pitch_exp_p);
      LOAD_MOTOR_VDES(motors->pitch, gimbal_vel_max);
    }
    if (motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->yaw, yaw_exp_tff);
    else {
      LOAD_MOTOR_PDES(motors->yaw, yaw_exp_p);
      LOAD_MOTOR_VDES(motors->yaw, gimbal_vel_max);
    }
    motors->pitch.AUX.EN = true;
    motors->yaw.AUX.EN = true;
#if OS_ENABLE
    extern xSemaphoreHandle xMutex_set_all_motor_output;
    if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE ||
        !OS_ENABLE) {
      rslt |= set_all_motor_output(GIMBAL_MOTORS);
      xSemaphoreGive(xMutex_set_all_motor_output);
    }
#else
    rslt |= set_all_motor_output(GIMBAL_MOTORS);
#endif
  } else {
    if (motors->pitch.work_mode == MIT_TT ||
        motors->pitch.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->pitch, 0);
    else
      LOAD_MOTOR_VDES(motors->pitch, 0);
    if (motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->yaw, 0);
    else
      LOAD_MOTOR_VDES(motors->yaw, 0);
    motors->pitch.AUX.EN = false;
    motors->yaw.AUX.EN = false;
    rslt |= HAL_OK;
  }
  return rslt;
}

/**
 * @brief 云台电机系控制
 * @note 需要借助轮毂电机来闭环 -->get_gimbal_angle
 * @attention 本框架使用的是电压控制GM6020
 * @attention 真·缓启动下cur量为raw_scale   缓启动下cur量为rad
 * 缓启动完成下cur量为abs_angle/RR
 */
HAL_StatusTypeDef gimbal_motor_ctrl(motors_t *motors, robot_t *robot,
                                    pid_struct_t pitch_pid[2],
                                    pid_struct_t yaw_pid[2]) {
  HAL_StatusTypeDef rslt = HAL_OK;
  motor_data_t *pitch_motor_pdata = &motors->pitch.real;
  motor_data_t *yaw_motor_pdata = &motors->yaw.real;
  robot_state_t *exp_state = robot->expected_state;
  robot_state_t *cur_state = robot->current_state;

  int16_t pitch_span = span_of(motors->pitch);
  int16_t yaw_span = span_of(motors->yaw);

  if (is_gimbal_reseted()) { // 用abs_angle/RR
    delta_ang_pitch =
        get_minor_arc(exp_state->pitch_ang, cur_state->pitch_ang, 2 * PI) /
        (2 * PI) * pitch_span;
    delta_ang_yaw =
        get_minor_arc(exp_state->yaw_ang, cur_state->yaw_ang, 2 * PI) /
        (2 * PI) * yaw_span;
  } else {
    if (is_gimbal_confirm_0degree) { // 用相对rad
      delta_ang_pitch =
          get_minor_arc(exp_state->pitch_ang, cur_state->pitch_ang, 2 * PI) /
          (2 * PI) * pitch_span;
      delta_ang_yaw =
          get_minor_arc(exp_state->yaw_ang, cur_state->yaw_ang, 2 * PI) /
          (2 * PI) * yaw_span;
    } else { // 用raw_scale
      delta_ang_pitch = get_minor_arc(exp_state->pitch_ang,
                                      pitch_motor_pdata->raw_scale, pitch_span);
      delta_ang_yaw = get_minor_arc(
          exp_state->yaw_ang, get_gimbal_angle(motors, &motors->yaw), yaw_span);
    }
  }
  float pitch_current_rpm =
      iir_filter_2(pitch_motor_pdata->rpm, GIMBAL_PITCH_FILTER2_RPM_CH);
  float yaw_current_rpm =
      iir_filter_2(yaw_motor_pdata->rpm, GIMBAL_YAW_FILTER2_RPM_CH);

  // 期望电压
  pitch_exp_tff =
      CLAMP(pid_dual_loop(pitch_pid, -delta_ang_pitch, pitch_current_rpm),
            GM6020_VOLT_MAX);
  yaw_exp_tff = CLAMP(pid_dual_loop(yaw_pid, -delta_ang_yaw, yaw_current_rpm),
                      GM6020_VOLT_MAX);

  pitch_exp_p = motors->pitch.AUX.xout.dat + pitch_motor_pdata->abs_angle +
                delta_ang_pitch / pitch_span * (2 * PI);
  yaw_exp_p = motors->yaw.AUX.xout.dat + yaw_motor_pdata->abs_angle +
              delta_ang_yaw / yaw_span * (2 * PI);

  // pitch_exp_tff = 0;   // debug
  // yaw_exp_tff = 0;     // debug
  if (is_motors_offline(GIMBAL_MOTORS)) {
    shutdown_motors_of_structure(GIMBAL_MOTORS);
    rslt |= HAL_ERROR;
  }
  if (robot->robot_control_flag) {
    if (motors->pitch.work_mode == MIT_TT ||
        motors->pitch.work_mode == QUAD_CURR)
      LOAD_MOTOR_LPFTFF(motors->pitch, pitch_exp_tff);
    else {
      LOAD_MOTOR_PDES(motors->pitch, pitch_exp_p);
      LOAD_MOTOR_VDES(motors->pitch, gimbal_vel_max);
    }
    if (motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
      LOAD_MOTOR_LPFTFF(motors->yaw, yaw_exp_tff);
    else {
      LOAD_MOTOR_PDES(motors->yaw, yaw_exp_p);
      LOAD_MOTOR_VDES(motors->yaw, gimbal_vel_max);
    }
    motors->pitch.AUX.EN = true;
    motors->yaw.AUX.EN = true;
#if OS_ENABLE
    extern xSemaphoreHandle xMutex_set_all_motor_output;
    if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE ||
        !OS_ENABLE) {
      rslt |= set_all_motor_output(GIMBAL_MOTORS);
      xSemaphoreGive(xMutex_set_all_motor_output);
    }
#else
    rslt |= set_all_motor_output(GIMBAL_MOTORS);
#endif
  } else {
    if (motors->pitch.work_mode == MIT_TT ||
        motors->pitch.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->pitch, 0);
    else
      LOAD_MOTOR_VDES(motors->pitch, 0);
    if (motors->yaw.work_mode == MIT_TT || motors->yaw.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->yaw, 0);
    else
      LOAD_MOTOR_VDES(motors->yaw, 0);
    motors->pitch.AUX.EN = false;
    motors->yaw.AUX.EN = false;
    rslt |= HAL_OK;
  }
  return rslt;
}

/**
 * @brief 基于底盘的电机系控制辅助函数
 */

static float last_ang[WHEELS_NUM] = {0};
static float last_gimbal_ang;
void reset_chassis_angle(void) {
  motors_t *motors = get_motors_ptr();
  for (uint8_t i = 0; i < WHEELS_NUM; i++)
    last_ang[i] = span_of(motors->_[WHEELS_NUM + i]);
}
static int16_t get_wheel_delta_ang(motor_t *pmotor, int16_t last_ang) {
  int16_t result = pmotor->real.raw_scale - last_ang;
  if ((result > 0) && ((int16_t)pmotor->real.rpm < 0))
    result -= span_of(*pmotor);
  else if ((result < 0) && ((int16_t)pmotor->real.rpm > 0))
    result += span_of(*pmotor);
  return result;
}

float get_gimbal_angle(motors_t *motors, motor_t *gimbal_pmotor) {
  static float z_angle = 0;
  int16_t wheel_delta_ang[WHEELS_NUM];
  int16_t gimbal_delta_ang;
  float wheel_delta_ang_sum = 0;
  uint8_t update_T_flag = 0; // 更新周期标志位
  motor_t *wheels_pmotors[WHEELS_NUM];

  for (uint8_t i = 0; i < WHEELS_NUM; i++) {
    wheels_pmotors[i] = &motors->_[WHEELS_MOTORS_INDEX + i];
    if (last_ang[i] == span_of(motors->_[WHEELS_MOTORS_INDEX + i]))
      update_T_flag = 1;
  }
  if (update_T_flag) {
    for (int i = 0; i < WHEELS_NUM; i++)
      last_ang[i] = wheels_pmotors[i]->real.raw_scale;
    last_gimbal_ang = gimbal_pmotor->real.raw_scale;
    z_angle = gimbal_pmotor->real.raw_scale;
  } else {
    for (int i = 0; i < WHEELS_NUM; i++) {
      wheel_delta_ang[i] = get_wheel_delta_ang(wheels_pmotors[i], last_ang[i]);
      wheel_delta_ang_sum += wheel_delta_ang[i];
      last_ang[i] = wheels_pmotors[i]->real.raw_scale;
    }
    gimbal_delta_ang = get_wheel_delta_ang(gimbal_pmotor, last_gimbal_ang);
    last_gimbal_ang = gimbal_pmotor->real.raw_scale;

    z_angle += (gimbal_delta_ang -
                ((D_WHEEL / 2) * (float)wheel_delta_ang_sum) / RR_M3508 /
                    (2 * (KXY_BACK + KXY_FRONT)) * K_CHASSIS_CAL);
    if (z_angle > ((float)span_of(*gimbal_pmotor)))
      z_angle -= span_of(*gimbal_pmotor);
    else if (z_angle < 0)
      z_angle += span_of(*gimbal_pmotor);
  }
  update_T_flag = 0;
  return z_angle;
}
