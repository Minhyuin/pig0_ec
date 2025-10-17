/**
 * @file shooter17.c
 *
 * @brief 17mm发射机构中间层+底层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "shooter.h"

#include "pid_lpf_param.h"
#include "robot.h"

extern CAN_HandleTypeDef FRIC_MOTORS_HCAN;
extern CAN_HandleTypeDef DIAL_MOTORS_HCAN;

extern uint8_t suggest_fire;
extern ext_game_robot_status_t game_robot_status;
extern ext_power_heat_data_t power_heat_data;

/*单发 控制目标*/
uint16_t single_shoot_wait_cnt = 0;
uint16_t single_shoot_wait_cnt_max = 50; // 单发角度稳定等待时间
float one_shoot_ang = ONE_SHOOT_ANG;
float delta_dial_ang = 0;
float fix_dial_ang = 0;
uint8_t single_shooting_flag = FINISH;
float cur_dial_rpm = 0, cur_dial_ang = 0;
bool lock_single_shoot = true;

/*防堵转*/
static int8_t stuck_dir = STUCK_FORWARD;
float stuck_current = STUCK_CURRENT;
float stuck_restore_rpm_inv_ad = STUCK_RESTORE_RPM_INV;
float stuck_restore_rpm_ad = STUCK_RESTORE_RPM;
static float stuck_restore_rpm = STUCK_RESTORE_RPM;
uint16_t dial_stuck_cnt = 0;
uint16_t stuck_max_time_cnt = STUCK_MAX_TIME_CNT;
uint16_t stuck_reverse_time_cnt = STUCK_REVERSE_TIME_CNT;
/**
 * @brief 连发防堵转处理函数
 * @brief 还可以再优化 目前对单发不启用
 * @retval output
 */
static float anti_stuck_output(pid_struct_t *rpm_pid, float curr, float cur_rpm,
                               float exp_rpm) {
  float output = 0;
  if (fabsf(exp_rpm) > STUCK_SPEED) {
    if (fabsf(cur_rpm) < STUCK_SPEED && fabsf(curr) > stuck_current) {
      // 检测到 电流过大
      if (dial_stuck_cnt < STUCK_MAX_TIME_CNT)
        // 检测到电流过大一段时间->>>>堵转
        dial_stuck_cnt++;
      else {
        asr_shooter_stuck_cmd();
        // 反转不了就正转 正转不了就反转 直到恢复顺畅 dial_stuck_cnt 变回0
        dial_stuck_cnt = stuck_reverse_time_cnt + stuck_max_time_cnt;
        if (stuck_dir == STUCK_FORWARD) { // 前时刻为正转
          stuck_dir = STUCK_REVERSE;
          stuck_restore_rpm = stuck_restore_rpm_inv_ad;
        } else if (stuck_dir == STUCK_REVERSE) {
          stuck_dir = STUCK_FORWARD;
          stuck_restore_rpm = stuck_restore_rpm_ad;
        }
      }
    } else if (dial_stuck_cnt <= stuck_max_time_cnt) {
      // 表示恢复顺畅
      dial_stuck_cnt = 0;
      stuck_dir = STUCK_FORWARD;
    }
    if (dial_stuck_cnt > STUCK_MAX_TIME_CNT) {
      exp_rpm = stuck_dir * stuck_restore_rpm;
      // STUCK_REVERSE_RPM * sign(sinf((pusher_stuck_cnt - STUCK_MAX_TIME_CNT)
      // / STUCK_REVERSE_TIME_CNT * (3 * PI))); TODO:平滑过渡消除阶跃
      dial_stuck_cnt--;
    }
    output = pid_calc(rpm_pid, -exp_rpm, cur_rpm);
  } else
    output = pid_calc(rpm_pid, 0, cur_rpm);
  return output;
}
static float anti_stuck_output_single(pid_struct_t *ang_pid, float curr,
                                      float cur_rpm, uint16_t *wait_cnt,
                                      float cur_del_ang, float exp_del_ang) {
  float output = 0;
  if (fabsf(cur_rpm) < STUCK_SPEED && fabsf(curr) > STUCK_CURRENT) {
    if (dial_stuck_cnt < SINGLE_STUCK_MAX_TIME_CNT)
      // 检测到电流过大一段时间->>>>堵转
      dial_stuck_cnt++;
    else {
      asr_shooter_stuck_cmd();
      *wait_cnt = 65535; // 等待时间置为最大 强制结束本次单发
    }
  }
  // output = pid_calc(ang_pid, exp_del_ang, cur_del_ang);
  output = pid_dual_loop(ang_pid, cur_del_ang - exp_del_ang, cur_rpm); //
  // 双环控制
  return output;
}

/*pid结构体*/
pid_struct_t fric_pid[2] = {0};
pid_struct_t dial_rpm_pid = {0};    // 连发对速度PID
pid_struct_t dial_ang_pid[2] = {0}; // 单发对角度PID

/*摩擦轮启用标志位*/
bool use_fric_flag = 0; // 摩擦轮是否需要转动

/*固定速度设置*/
float set_fric_rpm = 0;        // 摩擦轮设定(rpm)
float set_dial_rpm = DIAL_RPM; // 推弹速度设定(rpm)
float exp_dial_rpm = 0;        // DT7拨轮给的期望

/*电机输出编码值*/
float fric_tff[2] = {0};
float dial_tff = 0;

/*热量*/
int16_t remaining_heat = 0;
uint8_t heat_enable_flag = 0;

/*输入计数值*/
RAM_PERSIST float dial_cnt = 0;

static void shooter_state_clear(void) {
  pid_reset(&dial_rpm_pid);
  pid_reset(&dial_ang_pid[0]);
  pid_reset(&dial_ang_pid[1]);
  pid_reset(&fric_pid[LEFT]);
  pid_reset(&fric_pid[RIGHT]);

  dial_stuck_cnt = 0;

  single_shoot_wait_cnt = 0;
  lock_single_shoot = true;
  single_shooting_flag = FINISH;
  use_fric_flag = DISABLE;
  // fric_tff[LEFT] = fric_tff[RIGHT] = 0;
  set_dial_rpm = 0;
  dial_tff = 0;
}

/*远程整定DM 3519PID*/
float vdes_kp = 0.546579388f;
float vdes_ki = 0.00300000009f;
uint8_t write_fric_flag = 0;
uint8_t storage_fric_flag = 0;

/**
 * @brief 发射机构初始化
 * @retval None
 */
void shooter17_init(void) {
  motors_t *motors = get_motors_ptr();

  memcpy(&fric_pid[LEFT], &shoot_left_fric_pid_init_val, sizeof(pid_struct_t));
  memcpy(&fric_pid[RIGHT], &shoot_right_fric_pid_init_val,
         sizeof(pid_struct_t));

  memcpy(&dial_rpm_pid, &dial_rpm_pid_init_val, sizeof(pid_struct_t));
  memcpy(&dial_ang_pid, &dial_ang_pid_init_val, sizeof(pid_struct_t));

  //  CAN2
  //  ID  rxId    txId
  //  1   0x201   0x200
  DM_motor_register(&motors->L_fric, motor_model_DM_DM3519, VDES, COM_CAN,
                    (uint32_t *)&SHOOTER_MOTORS_HCAN, 1, RR_BARE_DM3519,
                    &L_fric_lpf_init_val, M_SHOOTER);
  //  2   0x202   0x200
  DM_motor_register(&motors->R_fric, motor_model_DM_DM3519, VDES, COM_CAN,
                    (uint32_t *)&SHOOTER_MOTORS_HCAN, 2, RR_BARE_DM3519,
                    &R_fric_lpf_init_val, M_SHOOTER);
  //  3   0x203   0x200
  RM_QUAD_motor_register(&motors->dial, motor_model_RM_QUAD_M2006, QUAD_CURR,
                         COM_CAN, (uint32_t *)&SHOOTER_MOTORS_HCAN, 3, RR_M2006,
                         &dial_lpf_init_val, M_SHOOTER);
  // 发射机构电机 放在fifo1
  can_user_init(&SHOOTER_MOTORS_HCAN,
                (uint32_t[]){
                    rx_stdid_of(motors->L_fric),
                    rx_stdid_of(motors->R_fric),
                    rx_stdid_of(motors->dial),
                    0,
                },
                1);
  dial_stuck_cnt = 0;
}

float cur_fric_rpm[2] = {0};
/**
 * @brief 17mm弹丸发射机构控制函数
 * @note 不存在base_shooter.c 这个函数本身即是base
 * @param dt 控制周期
 * @param robot 机器人结构体指针
 * @return HAL_StatusTypeDef
 *          HAL_StatusTypeDef:
 *          @arg HAL_OK 进程正常
 *          @arg HAL_ERROR 机构电机掉线
 */
HAL_StatusTypeDef shooter17_ctrl_loop(float dt, robot_t *robot) {
  HAL_StatusTypeDef rslt = HAL_OK;
  motors_t *motors = get_motors_ptr();

  if (write_fric_flag) {
    write_single_DM_motor_kp_asr(&motors->L_fric, vdes_kp, COM_CAN);
    write_single_DM_motor_kp_asr(&motors->R_fric, vdes_kp, COM_CAN);
    write_single_DM_motor_ki_asr(&motors->L_fric, vdes_ki, COM_CAN);
    write_single_DM_motor_ki_asr(&motors->R_fric, vdes_ki, COM_CAN);
    write_fric_flag = 0;
  }
  if (storage_fric_flag) {
    storage_single_DM_motor_kp_asr(&motors->L_fric, COM_CAN);
    storage_single_DM_motor_kp_asr(&motors->R_fric, COM_CAN);
    storage_single_DM_motor_ki_asr(&motors->L_fric, COM_CAN);
    storage_single_DM_motor_ki_asr(&motors->R_fric, COM_CAN);
    storage_fric_flag = 0;
  }

  cur_fric_rpm[LEFT] = motors->L_fric.real.rpm;
  cur_fric_rpm[RIGHT] = motors->R_fric.real.rpm;

  cur_dial_rpm = iir_filter_3(motors->dial.real.rpm / DIAL_RR, 10);
  cur_dial_ang = motors->dial.real.abs_angle;

  rc_ctrl_t *dt7_data = get_dt7_data_ptr();

  remaining_heat = game_robot_status.shooter_barrel_heat_limit -
                   power_heat_data.shooter_id1_17mm_cooling_heat;
  if (remaining_heat > MIN_SHOOT_HEAT ||
      game_robot_status.shooter_barrel_heat_limit == 65535 ||
      game_robot_status.shooter_barrel_heat_limit == 0)
    heat_enable_flag = ENABLE;
  else
    heat_enable_flag = DISABLE;

  // 控制期望弹速
  if (dt7_data) // 防wheel编码器损坏为0
    dial_cnt +=
        CLAMP((float)(dt7_data->wheel - 1024) * T_ACC_CNT / 660.0f - dial_cnt,
              S_CURVE_DIAL_ACC);
#if USE_DT7_WHEEL
  if (robot->keyboard_flag)
    exp_dial_rpm = -DIAL_RPM;
  else
    exp_dial_rpm = -fabsf(s_curve(DIAL_RPM, dial_cnt));
#else
  exp_dial_rpm = -DIAL_RPM;
#endif
#if (BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE) || !BOARDS_MODE
  if (remaining_heat > MIN_SHOOT_HEAT * 1.7f ||
      game_robot_status.shooter_barrel_heat_limit == 0xFFFF)
    exp_dial_rpm = -DIAL_RPM;
  else
    exp_dial_rpm = -DIAL_LOW_RPM;
#endif

  // 模态选择
  switch (robot->shooter_tag) {
  case S_STOP_MODE:
    shooter_state_clear();
    break;
  case S_MULTIPLE_MODE: // 无火控
    use_fric_flag = ENABLE;
    if (robot->fire_flag == ENABLE)
      set_dial_rpm = exp_dial_rpm;
    else
      set_dial_rpm = 0;
    break;
  case S_RESTRICT_MODE: // 带火控
    use_fric_flag = ENABLE;
    if (robot->fire_flag == ENABLE && suggest_fire && heat_enable_flag)
      set_dial_rpm = exp_dial_rpm;
    else
      set_dial_rpm = 0;
    break;
  case S_SINGLE_MODE:
    use_fric_flag = ENABLE;
    // 注意！ 这里对robot->fire_flag必须为"只读"操作
    if (robot->fire_flag == ENABLE && suggest_fire && heat_enable_flag &&
        lock_single_shoot) {
      single_shooting_flag = UNFINISH;
      fix_dial_ang = cur_dial_ang;
      lock_single_shoot = false; // 防止重复触发
    }
    if (single_shoot_wait_cnt > single_shoot_wait_cnt_max) {
      single_shoot_wait_cnt = 0;
      single_shooting_flag = FINISH;
      lock_single_shoot = true;
    }
    if (single_shooting_flag == UNFINISH) {
      delta_dial_ang = cur_dial_ang - fix_dial_ang;
      if (fabsf(delta_dial_ang - one_shoot_ang) < 0.05f)
        single_shoot_wait_cnt++; // 卡住基本是因为这里不加了
      else
        single_shoot_wait_cnt =
            single_shoot_wait_cnt < 1 ? 0 : single_shoot_wait_cnt--;
    } else {
      delta_dial_ang = one_shoot_ang;
      pid_reset(&dial_ang_pid[0]);
      pid_reset(&dial_ang_pid[1]);
    }
    break;
  default:
    break;
  }
  // 拨弹盘闭环
  if (robot->shooter_tag == S_SINGLE_MODE)
    dial_tff = anti_stuck_output_single(dial_ang_pid, motors->dial.real.current,
                                        cur_dial_rpm, &single_shoot_wait_cnt,
                                        delta_dial_ang, one_shoot_ang);
  else {
    dial_tff = anti_stuck_output(&dial_rpm_pid, motors->dial.real.current,
                                 cur_dial_rpm, -set_dial_rpm);
  }
  // 摩擦轮闭环
  float zero_number = 0;
  if (use_fric_flag == ENABLE)
    set_fric_rpm = FRIC_RPM;
  else {
    slope_following(&zero_number, &set_fric_rpm,
                    1000.f / SHOOTER_TIM_FREQ); // 此值决定停转速度
                                                // set_fric_rpm = 0;
  }
  // set_fric_rpm = 0; // debug
  fric_tff[LEFT] =
      CLAMP(pid_calc(&fric_pid[LEFT], -set_fric_rpm, cur_fric_rpm[LEFT]),
            DM3519_TRQE_MAX);
  fric_tff[RIGHT] =
      CLAMP(pid_calc(&fric_pid[RIGHT], set_fric_rpm, cur_fric_rpm[RIGHT]),
            DM3519_TRQE_MAX);

  // memset(fric_tff,0,sizeof(fric_tff));  // debug
  // memset(&dial_tff,0,sizeof(float));       // debug
  motors_type_t motors_type = SHOOTER_MOTORS;
  // motors_type_t motors_type = DIAL_MOTORS;
#if (BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE)
  motors_type = DIAL_MOTORS;
#elif (BOARDS_MODE && MACHINE_TYPE == GIMBAL_SLAVE)
  motors_type = FRIC_MOTORS;
#else
  motors_type = SHOOTER_MOTORS;
#endif

  if (is_motors_offline(motors_type)) {
    shutdown_motors_of_structure(motors_type);
    rslt |= HAL_ERROR;
  }

  if (robot->robot_control_flag) {
    LOAD_MOTOR_TFF(motors->dial, dial_tff);
		motors->dial.AUX.EN = true;
    if (motors->L_fric.work_mode == MIT_TT ||
        motors->L_fric.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->L_fric, fric_tff[LEFT]);
    else {
      LOAD_MOTOR_VDES(motors->L_fric, set_fric_rpm / 60.f * 2 * PI);
    }
    if (motors->R_fric.work_mode == MIT_TT ||
        motors->R_fric.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->R_fric, fric_tff[RIGHT]);
    else {
      LOAD_MOTOR_VDES(motors->R_fric, -set_fric_rpm / 60.f * 2 * PI);
    }
    motors->R_fric.AUX.EN = true;
    motors->L_fric.AUX.EN = true;
#if OS_ENABLE
    extern xSemaphoreHandle xMutex_set_all_motor_output;
    if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE) {
      rslt |= set_all_motor_output(motors_type);
      xSemaphoreGive(xMutex_set_all_motor_output);
    }
#else
    rslt |= set_all_motor_output(motors_type);
#endif
  } else {
    LOAD_MOTOR_TFF(motors->dial, 0);
    if (motors->L_fric.work_mode == MIT_TT ||
        motors->L_fric.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->L_fric, 0);
    else
      LOAD_MOTOR_VDES(motors->L_fric, 0);
    if (motors->R_fric.work_mode == MIT_TT ||
        motors->R_fric.work_mode == QUAD_CURR)
      LOAD_MOTOR_TFF(motors->R_fric, 0);
    else
      LOAD_MOTOR_VDES(motors->R_fric, 0);
    motors->R_fric.AUX.EN = false;
    motors->L_fric.AUX.EN = false;
#if OS_ENABLE
    extern xSemaphoreHandle xMutex_set_all_motor_output;
    if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE) {
      rslt |= shutdown_motors_of_structure(motors_type);
      xSemaphoreGive(xMutex_set_all_motor_output);
    }
#else
    rslt |= shutdown_motors_of_structure(motors_type);
#endif
  }
  return rslt;
}

/**
 * @brief 访问单发是否完成标志位
 * @retval uint8_t
 *         single_shooting_flag
 *         @arg FINISH 单发完成
 *         @arg UNFINISH 单发未完成
 */
uint8_t get_single_shooting_flag(void) { return single_shooting_flag; }
/**
 * @brief 获取热量是否允许发射标志位指针
 */
uint8_t *get_heat_enable_flag_ptr(void) { return &heat_enable_flag; }
