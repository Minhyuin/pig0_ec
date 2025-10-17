#ifndef _CHASSIS_H
#define _CHASSIS_H

#include "Base/rc.h"
#include "robot.h"

typedef struct {
  float v_x;
  float v_y;
  float w_z;
} chassis_state_t;

#define HIGH_ENERGY_PERCENT 0.3f

#define NORMAL_ACC (3.5f)
#define FLY_ACC 0.5f     // 对于平衡步兵该值需设低，非平衡步兵则该值设高
#define ECO_PRO_ACC 0.5f // 节能模式加速度
#define NOR_SIDEWAYS_ACC NORMAL_ACC
#define BAL_SIDEWAYS_ACC 0.5f
#define CREEP_ACC 1.5f
#define RUSH_ACC 99999.0f

#define NORMAL_WZ_ACC 8.f
#define SPIN_STOP_WZ_ACC 1.f
#define SPIN_START_WZ_ACC 3.f

#define MAX_VX_SPEED 3.0f
#define MAX_VY_SPEED 3.0f
#define MAX_WZ_RADPS 4.0f

#define MAX_RUSH_VX_SPEED 3.5f
#define MAX_RUSH_VY_SPEED 3.5f
#define MAX_RUSH_WZ_RPS 3.0f

#define MAX_VX_TIRED_SPEED 0.5f
#define MAX_VY_TIRED_SPEED 0.5f

#define MAX_CREEP_SPEED 4.0f

#define SPINNING_TEST_K 42
#define SPINNING_CYCLE_K 4.5f
#define SPINNING_BASE_K 1.0f
#define SPINNING_REDUCE_K 0.3f

#define SPINNING_MOV_GAIN_FORy 0.02f
#define SPINNING_MOV_GAIN_FORx 0.02f

#define SPINNING_SHUT_DEADBAND 0.4599609375f

void chassis_init(void);
void chassis_ctrl_loop(float dt, robot_t *robot);
uint8_t is_chassis_reseted(void);
uint8_t *get_chassis_reset_flag_ptr(void);

#endif
