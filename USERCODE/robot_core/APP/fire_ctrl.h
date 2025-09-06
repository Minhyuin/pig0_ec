#ifndef _FIRE_CTRL_H
#define _FIRE_CTRL_H

#include "../drv_conf.h"
#include HAL_INCLUDE

#define SHIFT_OR_SPIN_VYAW 2.5f
#define VYAW_BOUND_1 7.0f
#define VYAW_BOUND_2 10.0f

#define YAW_MOTOR_RES_SPEED 2.05f

// #define CHOOSE_SCOPE PI * 2.0f / 3.0f

#define LARGE_ARMOR_WIDTH 0.23
#define SMALL_ARMOR_WIDTH 0.11
#define ANGLE_MIN_ACCURACY 0.01f

enum e_aim_status {
  ARMOR = 0,
  BUFF,
  AIM_SHIFTING,
  AIM_SPINNING_SLOW,
  AIM_SPINNING_HIGH,
  AIM_SPINNING_MID,
  AIM_OUTPOST,
  AIM_BUFF
};

#define BUFF_R 0.7f

extern float aim_spin_status;
extern uint8_t aim_status, armor_choose;
extern uint8_t suggest_fire;

void expected_preview_calc(void);

#endif
