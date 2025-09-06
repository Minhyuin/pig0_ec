#include "./ui_ctrl.h"

#include "Base/UI/priority_ui.h"
#include "Base/UI/ui_def.h"
#include "Base/UI/ui_protocol.h"
#include "Base/rc.h"

#include "Base/power_ctrl.h"

#include "Base/vision.h"

#include "APP/behaviour.h"
#include "APP/robot.h"

#include "string.h"

uint32_t ui_ctrl_dwt_cnt = 0;
float ui_ctrl_dt = 1.f / UI_TIM_FREQ;
uint32_t ui_send_dwt_cnt = 0;
float ui_send_dt = 1.f / UI_SEND_FREQ;
bool ui_reset_req = false;

#define MAX_UI_RESET_CNT 20
bool lamda_ui_condition(void) {
  bool rslt = false;
  static uint16_t ui_reset_cnt = 0;
  rc_ctrl_t *rc = get_rc_data_ptr();
#if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER) || !BOARDS_MODE
  if (rc->rc.ch0 == 660 && rc->rc.ch1 == -660 && rc->rc.ch2 == -660 &&
      rc->rc.ch3 == -660) {
    ui_reset_req = true;
  } else {
    ui_reset_req = false;
  }
#endif
  if (ui_reset_req) {
    ui_reset_cnt++;
  }
  if (ui_reset_cnt >= MAX_UI_RESET_CNT) {
    rslt = true;
    ui_reset_cnt = 0;
  } else {
    rslt = false;
  }
  return rslt;
}

void UI_Init(void) {
  Init_Ui_Condition(lamda_ui_condition);
  // 这里会把ST图像发送出去
  Init_Ui_List(get_dynamic_ui_info(), get_dynamic_ui_num(),
               get_static_ui_info(), get_static_ui_num());
}
void UI_Send(void) {
  ui_send_dt = DWT_GetDeltaT(&ui_send_dwt_cnt);
  if (!ui_send_dt)
    ui_send_dt = 1.f / UI_SEND_FREQ;
  Ui_Priority_Send();
}

extern bool ui_initing_flag;
extern chassis_power_lim_t chassis_power_lim;

#define DY_INFO(ID) (get_dynamic_ui_info() + ID)
#define ST_INFO(ID) (get_static_ui_info() + ID)

#define UI_LOAD_TEXT(ID, TEXT)                                                 \
  strncpy(DY_INFO(ID)->ui_config.text, TEXT,                                   \
          sizeof(DY_INFO(ID)->ui_config.text))

#define UI_VEL_MAX 4.f

#define MAX_HP 500

extern ext_game_robot_HP_t game_robot_HP;
static float UI_duel_time_HP(uint8_t id) {
  uint16_t hp = 0;
  if (is_red_or_blue()) {
    // 表示自己为红方
    switch (id) {
    case 1:
      hp = game_robot_HP.blue_1_robot_HP;
      break;
    case 2:
      hp = game_robot_HP.blue_2_robot_HP;
      break;
    case 3:
      hp = game_robot_HP.blue_3_robot_HP;
      break;
    case 4:
      hp = game_robot_HP.blue_4_robot_HP;
      break;
    case 5:
      hp = game_robot_HP.blue_5_robot_HP;
      break;
    case 6:
      hp = game_robot_HP.blue_7_robot_HP;
      break;
    case 7:
      hp = game_robot_HP.blue_base_HP;
      break;
    case 0:
      hp = game_robot_HP.blue_outpost_HP;
      break;
    default:
      hp = 0;
      break;
    }
  } else {
    switch (id) {
    case 1:
      hp = game_robot_HP.red_1_robot_HP;
      break;
    case 2:
      hp = game_robot_HP.red_2_robot_HP;
      break;
    case 3:
      hp = game_robot_HP.red_3_robot_HP;
      break;
    case 4:
      hp = game_robot_HP.red_4_robot_HP;
      break;
    case 5:
      hp = game_robot_HP.red_5_robot_HP;
      break;
    case 6:
      hp = game_robot_HP.red_7_robot_HP;
      break;
    case 7:
      hp = game_robot_HP.red_base_HP;
      break;
    case 0:
      hp = game_robot_HP.red_outpost_HP;
      break;
    default:
      hp = 0;
      break;
    }
  }
  return (float)hp / MAX_HP;
}

/**
 * @brief UI队列更新
 * @warning
 * 高优先级元素高频进入队列会导致阻塞,推荐使用检测跳变或低频事件回调的方式驱动
 */
float start_ang_map;
void UI_Update(void) {
  if (ui_initing_flag) {
    return;
  }
  ui_ctrl_dt = DWT_GetDeltaT(&ui_ctrl_dwt_cnt);
  if (!ui_ctrl_dt)
    ui_ctrl_dt = 1.f / UI_TIM_FREQ;

  robot_t *robot = get_robot_ptr();
  vision_ctrl_t *vision = get_vision_ctrl_data_ptr();

  /*速度弧线*/
  float robot_spd =
      sqrtf((robot->current_state->v_x * robot->current_state->v_x +
             robot->current_state->v_y * robot->current_state->v_y));
  // 表示速度在UI_VEL_MAX占比映射到速度弧线上的角度
  float ui_real_vel_deg = robot_spd / UI_VEL_MAX *
                              (ST_INFO(ValRefArc)->ui_config.end_angel -
                               ST_INFO(ValRefArc)->ui_config.start_angel) +
                          ST_INFO(ValRefArc)->ui_config.start_angel;
  ;
  // 限制在弧线结束角度内
  ui_real_vel_deg = ui_real_vel_deg > ST_INFO(ValRefArc)->ui_config.end_angel
                        ? ST_INFO(ValRefArc)->ui_config.end_angel
                        : ui_real_vel_deg;
  // 注意在ui的圆弧系中 最高点为0点 顺时针增长
  DY_INFO(SpdVel)->ui_config.start_x =
      (uint32_t)(ST_INFO(ValRefArc)->ui_config.start_x +
                 cosf(ui_real_vel_deg * PI / 180.f - PI / 2.f) *
                     ST_INFO(ValRefArc)->ui_config.end_x);
  DY_INFO(SpdVel)->ui_config.start_y =
      (uint32_t)(ST_INFO(ValRefArc)->ui_config.start_y -
                 sinf(ui_real_vel_deg * PI / 180.f - PI / 2.f) *
                     ST_INFO(ValRefArc)->ui_config.end_x);
  Enqueue_Ui_For_Sending(DY_INFO(SpdVel));

  /* PITCH弧线 */
  float robot_gimbal_pitch =
      robot->gimbal_imu_data->eular.pitch / IMU_RANGE * 2.f * PI;
  float ui_real_pitch_deg = robot_gimbal_pitch / PI * 180.f;
  float max_delta_pitch_deg = (ST_INFO(PitchArc)->ui_config.end_angel -
                               ST_INFO(PitchArc)->ui_config.start_angel) /
                              2.f;
  ui_real_pitch_deg =
      (ui_real_pitch_deg > max_delta_pitch_deg)    ? max_delta_pitch_deg
      : (ui_real_pitch_deg < -max_delta_pitch_deg) ? -max_delta_pitch_deg
                                                   : ui_real_pitch_deg;
  DY_INFO(PitchAng)->ui_config.start_x =
      (uint32_t)(ST_INFO(PitchArc)->ui_config.start_x -
                 cosf(-ui_real_pitch_deg * PI / 180.f) *
                     ST_INFO(PitchArc)->ui_config.end_x);
  DY_INFO(PitchAng)->ui_config.start_y =
      (uint32_t)(ST_INFO(PitchArc)->ui_config.start_y +
                 sinf(-ui_real_pitch_deg * PI / 180.f) *
                     ST_INFO(PitchArc)->ui_config.end_x);
  Enqueue_Ui_For_Sending(DY_INFO(PitchAng));

  /*速度浮点*/
  DY_INFO(VelNum)->ui_config.float_num = robot_spd;
  Enqueue_Ui_For_Sending(DY_INFO(VelNum));

  /*PITCH浮点*/
  DY_INFO(PitchNum)->ui_config.float_num = robot_gimbal_pitch;
  Enqueue_Ui_For_Sending(DY_INFO(PitchNum));

  /*标记敌方血量*/
  if (robot->use_vision_flag) {
    if (vision->target == 1) {
      DY_INFO(EnemyHP)->ui_config.operate_type = MODIFY;
      float hp_ratio = UI_duel_time_HP(vision->id_num);
      float HP_del = (float)(MAX_HP_RIGHT_POS_X - MAX_HP_LEFT_POS_X);
      DY_INFO(EnemyHP)->ui_config.start_x =
          MAX_HP_LEFT_POS_X + HP_del / 2.f - hp_ratio / 2.f * HP_del;
      DY_INFO(EnemyHP)->ui_config.end_x =
          MAX_HP_RIGHT_POS_X - HP_del / 2.f + hp_ratio / 2.f * HP_del;
    } else {
      DY_INFO(EnemyHP)->ui_config.start_x = 959;
      DY_INFO(EnemyHP)->ui_config.end_x = 961;
    }
  }
  Enqueue_Ui_For_Sending(DY_INFO(EnemyHP));

  /*检查视觉是否在线*/
  if (vision->header) {
    DY_INFO(VisionState)->ui_config.operate_type = ADD;
  } else {
    DY_INFO(VisionState)->ui_config.operate_type = DELETE;
  }
  Enqueue_Ui_For_Sending(DY_INFO(VisionState));

  //  /*弹仓开关*/
  //  static enum _magazine_switch_flag last_switch_flag = UNCONTROL;
  //  if (last_switch_flag != robot->magazine_switch_flag) {
  //    if (robot->magazine_switch_flag == MAG_ON) {
  //      DY_INFO(Reload)->ui_config.operate_type = ADD;
  //    } else {
  //      DY_INFO(Reload)->ui_config.operate_type = DELETE;
  //    }
  //    Enqueue_Ui_For_Sending(DY_INFO(Reload));
  //    last_switch_flag = robot->magazine_switch_flag;
  //  }

  /*超电容量*/
  DY_INFO(CapEnergy)->ui_config.start_angel =
      CAP_ENERGY_ARC_END_ENG -
      (get_cap_ratio()) * (CAP_ENERGY_ARC_END_ENG - CAP_ENERGY_ARC_START_ENG);
  if (robot->use_supercap_flag)
    DY_INFO(CapEnergy)->ui_config.color = YELLOW;
  else
    DY_INFO(CapEnergy)->ui_config.color = WHITE;
  if (DY_INFO(CapEnergy)->ui_config.start_angel >
      DY_INFO(CapEnergy)->ui_config.end_angel - 5) {
    DY_INFO(CapEnergy)->ui_config.start_angel =
        DY_INFO(CapEnergy)->ui_config.end_angel - 5;
  }
  Enqueue_Ui_For_Sending(DY_INFO(CapEnergy));

  /*底盘总能耗(能量剩余)*/
  DY_INFO(ChassisEnergy)->ui_config.start_angel =
      CHASSIS_ENERGY_ARC_END_ENG -
      (get_chassis_energy() / CHASSIS_ENERGY_MAX) *
          (CHASSIS_ENERGY_ARC_END_ENG - CHASSIS_ENERGY_ARC_START_ENG);
  if (DY_INFO(ChassisEnergy)->ui_config.start_angel >
      DY_INFO(ChassisEnergy)->ui_config.end_angel - 5) {
    DY_INFO(ChassisEnergy)->ui_config.start_angel =
        DY_INFO(ChassisEnergy)->ui_config.end_angel - 5;
  }
  Enqueue_Ui_For_Sending(DY_INFO(ChassisEnergy));

  /*FOV */
  static uint8_t last_is_target = 0;
  if (last_is_target != vision->target) {
    if (vision->target) {
      DY_INFO(FOV)->ui_config.color = RED_BLUE;
      DY_INFO(FOV)->ui_config.width = 5;
    } else {
      DY_INFO(FOV)->ui_config.color = WHITE;
      DY_INFO(FOV)->ui_config.width = 1;
    }

    Enqueue_Ui_For_Sending(DY_INFO(FOV));
    last_is_target = vision->target;
  }

  /*摩擦轮开关*/
  if (robot->shooter_tag != robot->shooter_tag_last) {
    if (robot->shooter_tag == S_STOP_MODE)
      DY_INFO(FricSwitch)->ui_config.operate_type = ADD;
    else
      DY_INFO(FricSwitch)->ui_config.operate_type = DELETE;
    Enqueue_Ui_For_Sending(DY_INFO(FricSwitch));
  }

  /*小陀螺指示*/
  float wz_normalize = robot->current_state->w_z / MAX_WZ_RADPS;
  DY_INFO(SpinArc)->ui_config.start_angel = range_map_optim(
      DY_INFO(SpinArc)->ui_config.start_angel + wz_normalize * 10.f, 0.f,
      360.f);
  DY_INFO(SpinArc)->ui_config.end_angel =
      DY_INFO(SpinArc)->ui_config.start_angel + UI_SPINNING_DEG;
  if (DY_INFO(SpinArc)->ui_config.end_angel > 360)
    DY_INFO(SpinArc)->ui_config.end_angel = 360;
  if (robot->chassis_tag == C_SPIN_MODE) {
    Enqueue_Ui_For_Sending(DY_INFO(SpinArc));
  } else {
    static uint16_t ui_spin_wait_cnt = 0;
    ui_spin_wait_cnt++;
    DY_INFO(SpinArc)->ui_config.start_angel = 0;
    DY_INFO(SpinArc)->ui_config.end_angel = 1;
    do {
      Enqueue_Ui_For_Sending(DY_INFO(SpinArc));
      ui_spin_wait_cnt = 0;
    } while (ui_spin_wait_cnt > 30);
  }

  /*行车线*/
  // 采用线性变换 不符合透视原理 比较粗糙
  float gimbal_pitch_del = get_minor_arc(robot->gimbal_imu_data->eular.pitch,
                                         UI_IMU_PITCH_MAX, IMU_RANGE);
  float pitch_range =
      get_minor_arc(UI_IMU_PITCH_MIN, UI_IMU_PITCH_MAX, IMU_RANGE);
  DY_INFO(LeftMove)->ui_config.end_x =
      DOWN_MOVE_LEFT_POS_END_X +
      fabsf(gimbal_pitch_del / pitch_range) *
          (UP_MOVE_LEFT_POS_END_X - DOWN_MOVE_LEFT_POS_END_X);
  DY_INFO(LeftMove)->ui_config.end_y =
      DOWN_MOVE_LEFT_POS_END_Y +
      fabsf(gimbal_pitch_del / pitch_range) *
          (UP_MOVE_LEFT_POS_END_Y - DOWN_MOVE_LEFT_POS_END_Y);
  DY_INFO(RightMove)->ui_config.end_x =
      DOWN_MOVE_RIGHT_POS_END_X -
      fabsf(gimbal_pitch_del / pitch_range) *
          (DOWN_MOVE_RIGHT_POS_END_X - UP_MOVE_RIGHT_POS_END_X);
  DY_INFO(RightMove)->ui_config.end_y =
      DOWN_MOVE_RIGHT_POS_END_Y -
      fabsf(gimbal_pitch_del / pitch_range) *
          (DOWN_MOVE_RIGHT_POS_END_Y - UP_MOVE_RIGHT_POS_END_Y);
  Enqueue_Ui_For_Sending(DY_INFO(LeftMove));
  Enqueue_Ui_For_Sending(DY_INFO(RightMove));
}

void update_Ctag_ui_CBK(chassis_tag_t tag) {
  switch (tag) {
  case C_STOP_MODE:
    UI_LOAD_TEXT(ChassisState, "STOP");
    DY_INFO(ChassisState)->ui_config.color = RED_BLUE;
    break;
  case C_LAUNCH_MODE:
    UI_LOAD_TEXT(ChassisState, "LAUNCH");
    DY_INFO(ChassisState)->ui_config.color = ORANGE;
    break;
  case C_SPIN_MODE:
    UI_LOAD_TEXT(ChassisState, "SPIN");
    DY_INFO(ChassisState)->ui_config.color = YELLOW;
    break;
  case C_SIDEWAYS_MODE:
    UI_LOAD_TEXT(ChassisState, "SIDEWAYS");
    DY_INFO(ChassisState)->ui_config.color = PINK;
    break;
  case C_FLY_MODE:
    UI_LOAD_TEXT(ChassisState, "FLY");
    DY_INFO(ChassisState)->ui_config.color = WHITE;
    break;
  case C_SNIPE_MODE:
    UI_LOAD_TEXT(ChassisState, "SNIPE");
    DY_INFO(ChassisState)->ui_config.color = FUCHSIA;
    break;
  case C_RUSH_MODE:
    UI_LOAD_TEXT(ChassisState, "RUSH");
    DY_INFO(ChassisState)->ui_config.color = ORANGE;
    break;
  case C_TANK_MODE:
    UI_LOAD_TEXT(ChassisState, "TANK");
    DY_INFO(ChassisState)->ui_config.color = FUCHSIA;
    break;
  case C_FOLLOW_MODE:
    UI_LOAD_TEXT(ChassisState, "FOLLOW");
    DY_INFO(ChassisState)->ui_config.color = CYAN_BLUE;
    break;
  case C_CREEP_MODE:
    UI_LOAD_TEXT(ChassisState, "CREEP");
    DY_INFO(ChassisState)->ui_config.color = BLACK;
  }
  Enqueue_Ui_For_Sending(DY_INFO(ChassisState));
}

void update_Gtag_ui_CBK(gimbal_tag_t tag) {
  switch (tag) {
  case G_STOP_MODE:
    UI_LOAD_TEXT(GimbalState, "STOP");
    DY_INFO(GimbalState)->ui_config.color = RED_BLUE;
    break;
  case G_LAUNCH_MODE:
    UI_LOAD_TEXT(GimbalState, "LAUNCH");
    DY_INFO(GimbalState)->ui_config.color = ORANGE;
    break;
  case G_MANUAL_MODE:
    UI_LOAD_TEXT(GimbalState, "MANUAL");
    DY_INFO(GimbalState)->ui_config.color = FUCHSIA;
    break;
  case G_PATROL_MODE:
    UI_LOAD_TEXT(GimbalState, "PATROL");
    DY_INFO(GimbalState)->ui_config.color = PINK;
    break;
  case G_QUICKTURN_MODE:
    UI_LOAD_TEXT(GimbalState, "TURN");
    DY_INFO(GimbalState)->ui_config.color = BLACK;
    break;
  }
  Enqueue_Ui_For_Sending(DY_INFO(GimbalState));
}

void update_Stag_ui_CBK(shooter_tag_t tag) {
  switch (tag) {
  case S_STOP_MODE:
    UI_LOAD_TEXT(ShooterState, "STOP");
    DY_INFO(ShooterState)->ui_config.color = RED_BLUE;
    break;
  case S_SINGLE_MODE:
    UI_LOAD_TEXT(ShooterState, "SINGLE");
    DY_INFO(ShooterState)->ui_config.color = YELLOW;
    break;
  case S_RESTRICT_MODE:
    UI_LOAD_TEXT(ShooterState, "RESTRICT");
    DY_INFO(ShooterState)->ui_config.color = ORANGE;
    break;
  case S_MULTIPLE_MODE:
    UI_LOAD_TEXT(ShooterState, "MULT");
    DY_INFO(ShooterState)->ui_config.color = BLACK;
    break;
  }
  Enqueue_Ui_For_Sending(DY_INFO(ShooterState));
}
