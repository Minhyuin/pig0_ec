/**
 * @file shooter42.c
 *
 * @brief 42mm发射机构中间层+底层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "shooter.h"

#include "Algorithm/KF.h"
#include "Algorithm/filter.h"
#include "Algorithm/pid.h"
#include "Base/referee.h"
#include "Base/vision.h"
#include "Devices/MOTOR/motor_headers.h"
#include "boards_interact.h"
#include "bus_detect.h"
#include "drv_conf.h"
#include "fire_ctrl.h"
#include "pid_lpf_param.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void shooter42_init(void) { ; }
HAL_StatusTypeDef shooter42_ctrl_loop(float dt, robot_t *robot) {
  return HAL_OK;
}
