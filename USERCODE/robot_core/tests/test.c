/**
 * @file test.c
 *
 * @brief 测试电机的抽象机构
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "test.h"

#include "../APP/pid_lpf_param.h"
#include "../APP/robot.h"
#include "../Base/base_chassis.h"
#include "../Base/base_gimbal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

const pid_struct_t test_motor_pid_init_value = {.kp = 0.0f,
                                                .ki = 0.0f,
                                                .kd = 0.0f,
                                                .i_max =
                                                    C620_CURR_DATA_MAX / 50,
                                                .out_max = C620_CURR_DATA_MAX,
                                                .k_deadband = 500};
M_LPF_t test_motor_lpf = {
    .fc = M_TIM_FREQ,
    .ts = 1,
};
pid_struct_t test_motor_pid = {0};

float motor_output = 0;

void motor_test_init(void) {
#if USE_TEST_FILE
  motors_t *motors = get_motors_ptr();
  memcpy(&test_motor_pid, &test_motor_pid_init_value, sizeof(pid_struct_t));
  //  CAN1
  //  ID  rxId    txId
  //  1   0x201   0x200
  RM_QUAD_motor_register(&motors->test, motor_model_RM_QUAD_M3508, QUAD_CURR,
                         COM_CAN, (uint32_t *)&hcan1, 4, RR_M3508,
                         &test_motor_lpf, M_GIMBAL);
  can_user_init(&hcan1,
                (uint32_t[]){
                    rx_stdid_of(motors->test),
                    0,
                    0,
                    0,
                },
                1);
#endif
  ;
}
// float DM3519_cur_vel = 0;
float test_motor_tff = 0; // T_ff
float test_motor_v = 0;
float test_motor_p = 0;
float test_MIT_Kp = 0.0f;
float test_MIT_Kd = 0.0f;
HAL_StatusTypeDef motor_test_loop(float dt, robot_t *robot) {
  HAL_StatusTypeDef rslt = HAL_OK;
#if USE_TEST_FILE
  motors_t *motors = get_motors_ptr();
  extern float rocker_ry;
  //    if(robot->robot_control_flag == DISABLE)
  //        test_motor_tff = 0;
  //     else {
  // 				test_motor_tff = s_curve(10.0f,rocker_ry);
  //    }
  // DM3519_cur_vel = motors->test.real.omega;
  LOAD_MOTOR_TFF(motors->test, test_motor_tff);
  LOAD_MOTOR_VDES(motors->test, test_motor_v);
  LOAD_MOTOR_PDES(motors->test, test_motor_p);
  LOAD_MOTOR_KP(motors->test, test_MIT_Kp);
  LOAD_MOTOR_KD(motors->test, test_MIT_Kd);
  // motors->test.AUX.EN = true; // debug
  if (test_motor_tff || test_motor_v || test_motor_p)
    motors->test.AUX.EN = true;
  else
    motors->test.AUX.EN = false;
  //		motors->test.AUX.EN = true;

#if OS_ENABLE
  extern volatile xSemaphoreHandle xMutex_set_all_motor_output;
  if (xSemaphoreTake(xMutex_set_all_motor_output, portMAX_DELAY) == pdTRUE) {
    rslt |= set_all_motor_output(TEST_MOTORS);
    xSemaphoreGive(xMutex_set_all_motor_output);
  }
#else
  rslt |= set_all_motor_output(TEST_MOTORS);
#endif
  //		rslt |= read_single_DM_motor_xout(&motors->test);
  //		rslt |= read_single_DM_motor_pm(&motors->test);
#else
  rslt |= HAL_OK;
#endif
  return rslt;
}
