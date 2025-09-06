/**
 * @file chassis.c
 * @brief 底盘中间层
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "chassis.h"
#include "../Base/base_chassis.h"
#include "../Base/base_gimbal.h"
#include "pid_lpf_param.h"
#include "robot.h"

#define VX_INDEX 0
#define VY_INDEX 1
#define WZ_INDEX 2

extern CAN_HandleTypeDef CHASSIS_MOTORS_HCAN;

extern int8_t spin_direction;

extern float yaw_rad;
extern float pitch_rad;
extern gimbal_ctrl_type_t gimbal_ctrl_type;

/*底盘过程量*/
chassis_state_t raw_state;       // 换系前
chassis_state_t transform_state; // 换系后

/*底盘限制加速度*/
float max_vx_acc = NORMAL_ACC; // delta v 限制机体最大加速度
float max_vy_acc = NORMAL_ACC;
float max_wz_acc = SPIN_STOP_WZ_ACC;

/*是否需要以过渡态结束小陀螺的标志位*/
uint8_t need_pending_shut = DISABLE;

/*坦克模式取消换系标志位*/
uint8_t need_transform = ENABLE;

/*底盘缓启动标志*/
uint8_t chassis_reset_flag = 0;

/*侧身默认角度 弧度制*/
float sideways_ang = PI / 4.f; // PI/4 rad

/*小陀螺功率多项式拟合参数*/
float P_K[] = {1.37118243f, 1.62585395f, -0.01604226f, 0.00013414f,
               -0.00000037f};

/*底盘功率数据*/
chassis_power_lim_t chassis_power_lim = {
    .referee_power_buffer = 60,
    .expt_cap_energy_ratio = 0.8,
    .expt_referee_power_buffer = 20,
};

/*pid结构体*/
pid_struct_t chassis_wz_pid = {0};
pid_struct_t chassis_pid[3] = {0};
pid_struct_t chassis_follow_pid = {0};
pid_struct_t chassis_vision_follow_pid = {0};
pid_struct_t wheels_pid[GENERAL_NUM_OF_WHEEL] = {0};
pid_struct_t slider_pid[BALANCED_NUM_OF_WHEEL] = {0};

/*输入计数值*/
extern RAM_PERSIST float rocker_rx;
extern RAM_PERSIST float rocker_ry;

/*底盘跟随*/
int16_t chassis_follow_err_ang = 0;
int16_t yaw_delta = 0;
float move_vec_ang = 0;
float follow_yaw = 0;
float inv_yaw_rad = 0;
float closest_yaw_rad = 0;
#define DUAL_MOVE 0
#define ONE_MOVE 1
uint8_t move_type = DUAL_MOVE;
float ctrl_type_wz_k = IMU_RANGE / (2.f * PI);

static bool spin_direction_lock = 0;

// 底盘支持两个自由度运动时用这个
#define W_Z_FOLLOW                                                             \
  {                                                                            \
    move_type = DUAL_MOVE;                                                     \
    chassis_follow_err_ang =                                                   \
        (MACHINE_TYPE == CHASSIS_SLAVE)                                        \
            ? chassis_follow_err_ang                                           \
            : (int16_t)(yaw_ctrl_err -                                         \
                        get_minor_arc(yaw_rad, follow_yaw, 2.f * PI) *         \
                            ctrl_type_wz_k);                                   \
    if (robot->use_vision_flag)                                                \
      raw_state.w_z = pid_calc_deadband(&chassis_vision_follow_pid,            \
                                        (float)chassis_follow_err_ang, 0);     \
    else                                                                       \
      raw_state.w_z = pid_calc_deadband(&chassis_follow_pid,                   \
                                        (float)chassis_follow_err_ang, 0);     \
  }

// 底盘只支持一个自由度运动时用这个
#define OPTIMAL_W_Z_FOLLOW                                                     \
  {                                                                            \
    move_type = ONE_MOVE;                                                      \
    follow_yaw += move_vec_ang;                                                \
    inv_yaw_rad =                                                              \
        (yaw_rad - PI) < 0 ? (yaw_rad - PI + 2.f * PI) : (yaw_rad - PI);       \
    closest_yaw_rad =                                                          \
        fabsf(get_minor_arc(yaw_rad, follow_yaw, 2.f * PI)) >                  \
                fabsf(get_minor_arc(inv_yaw_rad, follow_yaw, 2.f * PI))        \
            ? inv_yaw_rad                                                      \
            : yaw_rad;                                                         \
    if (follow_yaw == yaw_rad_center)                                          \
      closest_yaw_rad = yaw_rad;                                               \
    if (!raw_state.v_x && raw_state.v_y)                                       \
      closest_yaw_rad = yaw_rad, follow_yaw = yaw_rad_center + PI / 2.f;       \
    chassis_follow_err_ang =                                                   \
        (MACHINE_TYPE == CHASSIS_SLAVE)                                        \
            ? chassis_follow_err_ang                                           \
            : (int16_t)(yaw_ctrl_err -                                         \
                        get_minor_arc(closest_yaw_rad, follow_yaw, 2.f * PI) * \
                            ctrl_type_wz_k);                                   \
    if (robot->use_vision_flag)                                                \
      raw_state.w_z = pid_calc_deadband(&chassis_vision_follow_pid,            \
                                        (float)chassis_follow_err_ang, 0);     \
    else                                                                       \
      raw_state.w_z = pid_calc_deadband(&chassis_follow_pid,                   \
                                        (float)chassis_follow_err_ang, 0);     \
  }

/*底盘俯仰角*/
static float get_chassis_pitch(motor_t *pitch, eular_t *eular) {
  float chassis_pitch = 0;
  float temp_c_p = 0;
  float yaw_motor_err = get_minor_arc(yaw_rad, yaw_rad_center, 2.f * PI);
  float del_pitch_ang =
      (PITCH_RR == 1)
          ? (GIMBAL_PITCH_MOTOR_CENTER - pitch->real.raw_scale)
          : (pitch_rad_center - (pitch->real.abs_angle / PITCH_RR)) /
                (2.f * PI) * IMU_RANGE;
  // 注: 前倾为正值，若不是则更改center和abs_angle顺序
  if (fabs(yaw_motor_err) > PI / 4 && fabs(yaw_motor_err) < 3 * PI / 4)
    temp_c_p = sinf(eular->roll * (2.f * PI) / IMU_RANGE) /
               cosf(PI / 2 - yaw_motor_err);
  else
    temp_c_p = sinf((eular->pitch - (del_pitch_ang)) * (2.f * PI) / IMU_RANGE) /
               cosf(yaw_motor_err);
  chassis_pitch = asinf(LIMIT_MIN_MAX(temp_c_p, -1, 1));
  if (isnan(chassis_pitch))
    chassis_pitch = 0;
  return chassis_pitch;
}

/**
 * @brief 生成底盘小陀螺速度函数
 * @note
 * 当前方案的速度函数为周期函数,振幅增益SPINNING_CYCLE_K,小陀螺增益SPINNING_BASE_K,底盘移动期望增益SPINNING_REDUCE_K
 * @param cur_state 机器人当前状态指针
 * @retval None
 */
static float get_spinning_speed(robot_state_t *exp_state, float max_wz) {
#if SPIN_DEBUG
  return max_wz *
         (1 -
          sqrtf(pow2f(exp_state->v_y / MAX_VY_SPEED) +
                pow2f(exp_state->v_x / MAX_VX_SPEED)) *
              SPINNING_REDUCE_K); // 6.f;
                                  // //(float)(game_robot_status.chassis_power_limit
                                  // -
                                  //  SPINNING_TEST_K);
#else
  static uint16_t spin_alg_cnt = 1;
  spin_alg_cnt = spin_alg_cnt > 65534 ? 1 : spin_alg_cnt + 1;
  return (sinf(log(spin_alg_cnt)) * SPINNING_CYCLE_K +
          sqrtf(chassis_power_lim.power_limit) * SPINNING_BASE_K -
          sqrtf(powf(exp_state->v_y, 2) + powf(exp_state->v_x, 2)) *
              SPINNING_REDUCE_K) *
         0.10467f * 0.24f;
#endif
}
/**
 * @brief 底盘状态清零函数
 */
static void chassis_state_clear(chassis_state_t *state) {
  pid_reset(&chassis_pid[VX_INDEX]);
  pid_reset(&chassis_pid[VY_INDEX]);
  pid_reset(&chassis_pid[WZ_INDEX]);
  pid_reset(&chassis_follow_pid);
  pid_reset(&chassis_vision_follow_pid);

  need_pending_shut = DISABLE;
  // shutdown_all_motor();
  state->v_x = 0, state->v_y = 0, state->w_z = 0;
}

/**
 * @brief 底盘机构初始化函数
 * @retval None
 */
void chassis_init(void) {
  motors_t *motors = get_motors_ptr();

  for (int i = 0; i < GENERAL_NUM_OF_WHEEL; i++) {
    memcpy(&wheels_pid[i], &wheels_pid_init_val, sizeof(pid_struct_t));
  }
#if (BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE) || !BOARDS_MODE
  //  CAN1
  //  ID  rxId    txId
  //  1   0x201   0x200
  RM_QUAD_motor_register(&motors->RF_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         1, RR_M3508, &RF_wheel_lpf_init_val, M_CHASSIS);
  //  2   0x202   0x200
  RM_QUAD_motor_register(&motors->LF_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         2, RR_M3508, &LF_wheel_lpf_init_val, M_CHASSIS);
  //  3   0x203   0x200
  RM_QUAD_motor_register(&motors->LB_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         3, RR_M3508, &LB_wheel_lpf_init_val, M_CHASSIS);
  //  4   0x204   0x200
  RM_QUAD_motor_register(&motors->RB_wheel, motor_model_RM_QUAD_M3508,
                         QUAD_CURR, COM_CAN, (uint32_t *)&CHASSIS_MOTORS_HCAN,
                         4, RR_M3508, &RB_wheel_lpf_init_val, M_CHASSIS);

  can_user_init(&CHASSIS_MOTORS_HCAN, (uint32_t[]){CAP_RESPONSE_ID, 0, 0, 0},
                1);

  can_user_init(&CHASSIS_MOTORS_HCAN,
                (uint32_t[]){rx_stdid_of(motors->RF_wheel),
                             rx_stdid_of(motors->LF_wheel),
                             rx_stdid_of(motors->LB_wheel),
                             rx_stdid_of(motors->RB_wheel)},
                0);

#endif

  /*公有部分*/
  memcpy(&chassis_pid[VX_INDEX], &chassis_vx_pid_init_val,
         sizeof(pid_struct_t));
  memcpy(&chassis_pid[VY_INDEX], &chassis_vy_pid_init_val,
         sizeof(pid_struct_t));
  memcpy(&chassis_pid[WZ_INDEX], &chassis_wz_pid_init_val,
         sizeof(pid_struct_t));

  memcpy(&chassis_follow_pid, &chassis_follow_pid_init_val,
         sizeof(pid_struct_t));
  memcpy(&chassis_vision_follow_pid, &chassis_vision_follow_pid_init_val,
         sizeof(pid_struct_t));
}
/**
 * @brief 底盘控制函数
 * @param rc_recv 遥控器指令结构体指针
 * @param robot 机器人结构体指针
 * @retval None
 */
float yaw_ctrl_err = 0;
void chassis_ctrl_loop(float dt, robot_t *robot) {
  motors_t *motors = get_motors_ptr();
  robot_state_t *exp_state = robot->expected_state,
                *cur_state = robot->current_state;

  float yaw_angle_range = span_of(motors->yaw);
  float pitch_angle_range = span_of(motors->pitch);

// 若无底盘imu，则立体几何计算chassis_pitch 单位rad
#if USE_CHASSIS_IMU
  float chassis_pitch =
      robot->chassis_imu->eular.pitch / (IMU_RANGE) * 2.f * PI;
#else
  float chassis_pitch =
      get_chassis_pitch(&motors->pitch, &robot->gimbal_imu_data->eular);
#endif

#if USE_REFEREE
  chassis_power_lim.power_limit = game_robot_status.chassis_power_limit,
  chassis_power_lim.referee_power_buffer = power_heat_data.chassis_power_buffer;

#else
  chassis_power_lim.power_limit = 150.f,
  chassis_power_lim.referee_power_buffer = 60.f;
#endif
  if (robot->use_supercap_flag)
    chassis_power_lim.expt_cap_energy_ratio = 0.2f;
  else
    chassis_power_lim.expt_cap_energy_ratio = 0.85f;

  // 输入值映射到机体期望
  float max_vx = MAX_VX_SPEED;
  float max_vy = MAX_VY_SPEED;
  float max_wz = MAX_WZ_RADPS;
  if (robot->use_supercap_flag) {
    max_vx = MAX_RUSH_VX_SPEED;
    max_vy = MAX_RUSH_VY_SPEED;
    max_wz = MAX_WZ_RADPS; // MAX_RUSH_WZ_RPS;
  }
  raw_state.v_x = s_curve(max_vx, rocker_rx);
  raw_state.v_y = s_curve(max_vy, rocker_ry);
#if !USE_REFEREE
  chassis_power_lim.real_cap_energy_ratio = 1.f;
#endif
  if (get_cap_ratio() > HIGH_ENERGY_PERCENT) {
    max_vx_acc = max_vy_acc = NORMAL_ACC;
  } else {
    max_vx_acc = max_vy_acc = NORMAL_ACC;
  }

  if (gimbal_ctrl_type == USE_IMU)
    yaw_ctrl_err =
        get_minor_arc(cur_state->yaw_ang, exp_state->yaw_ang, IMU_RANGE),
    ctrl_type_wz_k = IMU_RANGE / (2.f * PI);
  else if (gimbal_ctrl_type == USE_MOTOR) // TODO:只靠电机数据做不到底盘跟随
    yaw_ctrl_err = get_minor_arc(get_gimbal_angle(motors, &motors->yaw) /
                                     span_of(motors->yaw) * 2.f * PI,
                                 exp_state->yaw_ang, 2.f * PI),
    ctrl_type_wz_k = 1.f;

  // 计算底盘跟随的相对头部坐标系yaw角
  move_vec_ang = atan2f(raw_state.v_y, raw_state.v_x);
  if (robot->robot_basic_mode == SEMIAUTO_MODE)
    follow_yaw = range_map(yaw_rad_center + PI / 4.f, 0, 2 * PI);
  else
    follow_yaw = yaw_rad_center;
  yaw_delta = (MACHINE_TYPE == CHASSIS_SLAVE)
                  ? yaw_delta
                  : get_minor_arc(yaw_rad, yaw_rad_center, 2.f * PI) /
                        (2.f * PI) * 32767.f;
  // 根据tag变换w_z及acc  作为从机 不允许被动切换tag(不可在此处调用update_tag)
  need_transform = ENABLE;
  switch (robot->chassis_tag) {
  case C_STOP_MODE: // 断控模式
    chassis_state_clear(&raw_state);
    break;
  case C_LAUNCH_MODE: // 缓启动模式:根据不同机器人作出反应
    chassis_state_clear(&raw_state);
    break;
  case C_FOLLOW_MODE: // 跟随模式:默认的底盘跟随
    spin_direction_lock = 0;
    if (need_pending_shut) {
      yaw_ctrl_err = 0;
      max_wz_acc = SPIN_STOP_WZ_ACC;
    }
    if (abs(yaw_delta) < SPINNING_SHUT_DEADBAND) {
      need_pending_shut = DISABLE;
      max_wz_acc = NORMAL_WZ_ACC;
    }
    W_Z_FOLLOW;
    break;
  case C_TANK_MODE: // 坦克模式:底盘不跟随头
    raw_state.w_z = 0;
    need_transform = ENABLE;
    break;
  case C_FLY_MODE: // 飞坡模式:不同机器人的飞坡需要不同的特殊加速度
    if (robot->rush_flag)
      max_vx_acc = max_vy_acc = RUSH_ACC;
    else
      max_vx_acc = max_vy_acc = FLY_ACC;
    W_Z_FOLLOW;
    break;
  case C_SIDEWAYS_MODE: // 侧身模式:不同机器人侧对角度和加速度不同
    max_vx_acc = max_vy_acc = NOR_SIDEWAYS_ACC;
    follow_yaw += sideways_ang;
    W_Z_FOLLOW;
    break;
  case C_SPIN_MODE: // 小陀螺模式:底盘不跟随头
    if (!spin_direction_lock) {
      spin_direction = -spin_direction;
      spin_direction_lock = 1;
    }
    need_pending_shut = ENABLE;
    raw_state.v_x +=
        cur_state->v_x * SPINNING_MOV_GAIN_FORx * -sign(raw_state.v_x);
    raw_state.v_y +=
        cur_state->v_y * SPINNING_MOV_GAIN_FORy * -sign(raw_state.v_y);
    raw_state.w_z = spin_direction * get_spinning_speed(exp_state, max_wz);
    max_wz_acc = SPIN_START_WZ_ACC;
    break;
  case C_CREEP_MODE: // 匍匐模式:不同机器人需要加速度不同
    max_vx_acc = max_vy_acc = CREEP_ACC;
    W_Z_FOLLOW;
    break;
  case C_SNIPE_MODE: // 狙击模式:wasd输入不能改变底盘
    chassis_state_clear(&raw_state);
    break;
  default:
    chassis_state_clear(&raw_state);
    break;
  }
  // 换系 使云台正前方为raw_state的正前方
  if (need_transform) {
    if (move_type == DUAL_MOVE) {
      float sin_yaw = sinf(yaw_delta / 32767.f * 2.f * PI);
      float cos_yaw = cosf(yaw_delta / 32767.f * 2.f * PI);
      transform_state.v_x = sin_yaw * raw_state.v_y + cos_yaw * raw_state.v_x;
      transform_state.v_y = cos_yaw * raw_state.v_y - sin_yaw * raw_state.v_x;
    } else {
      transform_state.v_x =
          (raw_state.v_x) ? (sign(cosf(move_vec_ang)) *
                             sqrtf(pow2f(raw_state.v_x) + pow2f(raw_state.v_y)))
                          : (raw_state.v_y);
      transform_state.v_y = 0;
    }
    float P = chassis_power_lim.power_limit;
    LIMIT_MIN_MAX(P, 0.f, 150.f);
    float wz_with_power_set = (P_K[4] * pow4f(P) + P_K[3] * pow3f(P) +
                               P_K[2] * pow2f(P) + P_K[1] * P + P_K[0]) /
                              60.f * 2.f * PI;
    LIMIT_MIN_MAX(wz_with_power_set, 0.f, MAX_WZ_RADPS);
    if (wz_with_power_set > fabsf(raw_state.w_z))
      transform_state.w_z = raw_state.w_z; // 没到功率墙,不限
    else
      transform_state.w_z = wz_with_power_set * sign(raw_state.w_z);
  } else {
    memcpy(&transform_state, &raw_state, sizeof(chassis_state_t));
  }

  // 微分法限加速度
  exp_state->v_x = transform_state.v_x;
  exp_state->v_y = transform_state.v_y;
  exp_state->w_z = transform_state.w_z;

#if (BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE) || !BOARDS_MODE
  // exp_state->w_z = 0; // debug
  general_chassis_base_ctrl(robot, motors, &chassis_power_lim, wheels_pid,
                            chassis_pid);
#endif
}

/**
 * @brief 判断底盘是否缓启动完成函数
 * @retval None
 */
uint8_t is_chassis_reseted(void) {
#if USE_CHASSIS_RESET
  if (*get_gimbal_reset_flag_ptr() == UNFINISH)
    chassis_reset_flag = UNFINISH;
  else
    chassis_reset_flag = FINISH;
#else
  chassis_reset_flag = FINISH;
#endif
  return chassis_reset_flag;
}
uint8_t *get_chassis_reset_flag_ptr(void) {
  is_chassis_reseted();
  return &chassis_reset_flag;
}
