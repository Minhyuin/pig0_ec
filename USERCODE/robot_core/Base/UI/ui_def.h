#ifndef _UI_DEF_H_
#define _UI_DEF_H_

#include "stdint.h"

#include "./priority_ui.h"
#include "./ui_protocol.h"
#include <stdint.h>

#define CAP_ENERGY_ARC_START_ENG 60
#define CAP_ENERGY_ARC_END_ENG 119

#define CHASSIS_ENERGY_ARC_START_ENG 60
#define CHASSIS_ENERGY_ARC_END_ENG 118

// 真实弹道
#define REAL_AIM_X (960 - 22)

// 行车线
// 低头到最极限角度的参数
#define UI_IMU_PITCH_MAX 209.375f
#define DOWN_MOVE_LEFT_POS_START_X 350
#define DOWN_MOVE_LEFT_POS_START_Y 0
#define DOWN_MOVE_LEFT_POS_END_X 630
#define DOWN_MOVE_LEFT_POS_END_Y 400
#define DOWN_MOVE_RIGHT_POS_START_X 1570
#define DOWN_MOVE_RIGHT_POS_START_Y 0
#define DOWN_MOVE_RIGHT_POS_END_X 1265
#define DOWN_MOVE_RIGHT_POS_END_Y 400

// 抬头到极限角度的参数
#define UI_IMU_PITCH_MIN -627.f
#define UP_MOVE_LEFT_POS_START_X 350
#define UP_MOVE_LEFT_POS_START_Y 0
#define UP_MOVE_LEFT_POS_END_X 830
#define UP_MOVE_LEFT_POS_END_Y 200
#define UP_MOVE_RIGHT_POS_START_X 1570
#define UP_MOVE_RIGHT_POS_START_Y 0
#define UP_MOVE_RIGHT_POS_END_X 1065
#define UP_MOVE_RIGHT_POS_END_Y 200

// 小陀螺指示
#define UI_SPINNING_DEG 30

// 敌方血量
#define MAX_HP_LEFT_POS_X (959 - 300)
#define MAX_HP_RIGHT_POS_X (961 + 300)

typedef enum {
  SpdVel,
  PitchAng,

  // Reload,

  VelNum,
  PitchNum,

  EnemyHP,

  CapEnergy,
  FOV,

  GimbalState,
  ChassisState,
  ShooterState,
  VisionState,
  ChassisEnergy,
  FricSwitch,

  LeftMove,
  RightMove,

  SpinArc,

  DYNAMIC_UI_NUM,
} dynamic_ui_cnt_e;

typedef enum {
  PowerBottomLine,
  ChassisBottomLine,
  // PowerTopLine,
  // ChassisTopLine,

  RealAttVerticalLine,
  RealAttHorizontalLine,
  Gimbal,
  Chassis,
  Shooter,
  Vision,
  ValRefArc,
  PitchArc,

  EnergyTheshold,
  PowerTheshold,
  VelTheshold,
  PitchTheshold,

  BottomLine1,
  BottomLine3,
  BottomLine5,
  EN,
  PO,

  PowerStaticArc,
  EnergyStaticArc,

  EnemyHPRect,

  STATIC_UI_NUM,
} static_ui_cnt_e;

ui_info_t *get_dynamic_ui_info(void);
uint32_t get_dynamic_ui_num(void);
ui_info_t *get_static_ui_info(void);
uint32_t get_static_ui_num(void);

#endif // !MY_UI_H_
