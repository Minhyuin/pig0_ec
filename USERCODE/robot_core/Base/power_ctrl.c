/**
 * @file power_ctrl.c
 *
 * @brief 功率限制
 *
 *
 * @copyright SCNU-PIONEER (c) 2023-2024
 *
 */
#include "power_ctrl.h"

#include <math.h>
#include <string.h>

#include "APP/robot.h"
#include "Base/super_cap.h"
#include "Devices/MOTOR/motor_headers.h"

#define P_idle (3.4418f) / 4
#define Kn (float)(0.013201)
float Kn_inv = Kn;
#define MLC (float)(2.2543e-05) // Mechanical loss coefficient
#define ESR (float)(0.29283)    // equivalent series resistance

pid_struct_t self_power_base_ratio_pid;
pid_struct_t self_power_limit_pid;
pid_struct_t self_power_unlimit_pid;

float real_cap_ratio = 0;
float real_chassis_energy = 0; // 已使用能量

float get_cap_ratio(void) { return real_cap_ratio; }
float *get_cap_ratio_ptr(void) { return &real_cap_ratio; }
float get_chassis_energy(void) { return real_chassis_energy; }
float *get_chassis_energy_ptr(void) { return &real_chassis_energy; }

float _cap_energy = 0;
/**
 * @brief Get the M3508 power object
 *
 * @param omega 输出端角速度(弧度制,rad/s)
 * @param Iq 输出端转矩电流(公制,A)
 * @return float
 */
float get_chasiss_motors_power(float omega, float Iq) {
  return (Kn * omega * Iq + MLC * omega * omega + ESR * Iq * Iq + P_idle);
  // return (Kn * omega * Iq + MLC * omega * omega);
};

/**
 * @brief 依据底盘电机的功率模型,计算出理论上设定功率对应的设定的电流
 *
 * @param P 设定功率
 * @param omega 电机输出端的角速度(弧度制,rad/s)
 * @param dirc 期望的旋转方向(对应期望转矩电流方向)
 * @return float 为计算得到的设定电流
 */
float get_chassis_motors_Iq(float P, float omega, int8_t dirc) {
  float omegaS2 = omega * omega;
  float RS2 = 0, Iq1 = 0, Iq2 = 0;
  RS2 = Kn * Kn * omegaS2 - 4 * ESR * MLC * omegaS2 + 4 * ESR * P -
        4 * ESR * P_idle;
  if (RS2 < 0) {
    RS2 = 0;
  }
  Iq1 = (-(Kn * omega - sqrtf(RS2)) / (2 * ESR));
  //-(-sqrt(rs2)),为正的可能大
  Iq2 = (-(Kn * omega + sqrtf(RS2)) / (2 * ESR));
  //+(-sqrt(rs2)),为负的可能大
  if (dirc >= 0)
    return Iq1;
  else
    return Iq2;

  // float Iq1 = 0;
  // if (omega == 0)
  //   Iq1 = dirc * MAX_CURRENT_LIMIT;
  // else
  //   Iq1 = P / (Kn_inv * omega);
  // return Iq1;
}

float soft_HP_MAX = 200;
float soft_HP = 200;
/**
 * @brief 软件热量限制,可在无裁判系统且拥有功率板调用
 *
 * @param pow_lim 功率限制结构体
 * @param power_limit 设定限制输入功率
 */
void update_soft_chassis_energy_buffer(chassis_power_lim_t *pow_lim) {
  cap_data_t *cap_data = get_cap_data();
  float referee_power_buffer = pow_lim->referee_power_buffer;
  float power_in = cap_data->input_voltage * cap_data->input_current;
  float power_limit = pow_lim->power_limit;

  static uint32_t tickstart = 0;
  uint32_t delta_tick = CLAMP(HAL_GetTick() - tickstart, 100);
  tickstart = HAL_GetTick();
  // 不会真的有人会用周期大于0.1s的频率控制底盘吧
  float t = delta_tick / 1000.f;
  float E_charge = (power_limit - power_in) * t;
  referee_power_buffer += E_charge;

  if (referee_power_buffer < 0.f) { // 超功率力!
    float K = (power_limit - power_in) / power_limit;
    float N = (K <= 0.1f) ? 0.1f : //
                  ((K <= 0.2f) ? 0.2f : 0.4f);
    soft_HP -= soft_HP_MAX * N * t;
  }
  pow_lim->power_limit = power_limit;
  pow_lim->referee_power_buffer = referee_power_buffer;
  LIMIT_MIN_MAX(pow_lim->referee_power_buffer, 0, 60.f);
}
void update_cap_buffer(void) {}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
uint8_t cap_state = 1; // 为1则开启电容组限制输入功率 0关闭
float power_limit_ratio = 1.f;
motor_power_parameter_calibrate_bag_t MPPCB = {0};
pid_struct_t referee_buffer_limit_pid = {.kp = 0.1,
                                         .out_max = 1};     // 0.1就对应正负10J
pid_struct_t power_limit_pid = {.kp = 300, .out_max = 400}; // 单位:W
//{.kp=500,.ki=1,.i_max=100,.out_max=500};//<-年轻人你渴望力量(炸鸡)嘛
LPF_t set_charge_power_filter = {.ts = 0.001f, .fc = 10.f};
float k_feedforward = 0.01;
float set_charge_power = 0;

/**
 * @brief 底盘功率分配,其中包含了发送电容
 *
 * @param pow_lim
 */
void chassis_power_distributor(chassis_power_lim_t *pow_lim) {
  cap_data_t *cap_data = get_cap_data();
  static uint32_t tickstart = 0;
  uint32_t delta_tick = CLAMP(HAL_GetTick() - tickstart, 100);
  // 不会真的有人会用周期大于0.1s的频率控制底盘吧
  tickstart = HAL_GetTick();
  ///>>> 确定可调配的功率值
  float k_referee_buffer = // 通过改变闭环期望电容组剩余能量进而实现稳定缓冲能量
      pid_calc(&referee_buffer_limit_pid,          //
               pow_lim->expt_referee_power_buffer, //
               pow_lim->referee_power_buffer);
  LIMIT_MIN_MAX(k_referee_buffer, -1.f, 1.f);
  // 缓冲能量过低->k_referee_buffer向着1上升->缩小期望与实际电容剩余能量差->减低输出功率
  // 缓冲能量过高->k_referee_buffer向着-1下降->提高期望与实际电容剩余能量差->减低输出功率
  extern float VCAP_MAX;
  pow_lim->real_output_power =
      cap_data->input_voltage * cap_data->output_current;
  pow_lim->real_chassis_energy -=
      pow_lim->real_output_power * delta_tick / 1000.f;
  real_chassis_energy = pow_lim->real_chassis_energy =
      pow_lim->real_chassis_energy > 0 ? pow_lim->real_chassis_energy : 0;
  float k = pow2f(VCAP_MAX) - pow2f(VCAP_MIN);
  pow_lim->real_cap_energy_ratio = // 实际电容剩余能量百分比
      (pow2f(cap_data->cap_voltage) - pow2f(VCAP_MIN)) / k;

  real_cap_ratio = pow_lim->real_cap_energy_ratio; // 获取剩余能量百分比

  float ratio_error =
      pow_lim->real_cap_energy_ratio - pow_lim->expt_cap_energy_ratio;
  float expt_cap_energy_ratio =
      pow_lim->expt_cap_energy_ratio + k_referee_buffer * ratio_error;

  LIMIT_MIN_MAX(expt_cap_energy_ratio, 0.f, 1.f);
  pow_lim->set_total_power = //
      pow_lim->power_limit - //
      pid_calc(&power_limit_pid, expt_cap_energy_ratio,
               pow_lim->real_cap_energy_ratio);
  LIMIT_MIN_MAX(pow_lim->set_total_power, pow_lim->power_limit,
                cap_data->cap_voltage * CAP_CURR_MAX + pow_lim->power_limit);
  // 电容组电压过低时,就应该自行降低输出功率以避免输入超功率

  /**参数设定+debug**/
  ///>>> null

#if ENABLE_MOTOR_POWER_PARAM_CALIBR_FLAG
  /**参数标定模式**/
  motor_response_msg_t *all_motor_data = get_motor_data_ptr();
  float real_power = cap_data->input_voltage * cap_data->input_current;
  float fitted_power = get_chasiss_motors_power( //
      pow_lim->real_omega[i], pow_lim->real_Iq[i]);
  MPPCB = (motor_power_parameter_calibrate_bag_t){
      .real_power = real_power,                  //
      .set_ctrl_value = pow_lim->raw_expt_Iq[0], //
      .real_ctrl_value = pow_lim->real_Iq[0],    //
      .real_omega = pow_lim->real_omega[0],      //
      .fitted_power = fitted_power,
  };
  pow_lim->set_Iq[0] = pow_lim->raw_expt_Iq[0];
  pow_lim->set_Iq[1] = 0;
  return;
#else
  /**软件开环限制输出原始功率**/
  ///>>> 获取参数
  float *raw_expt_Iq = pow_lim->raw_expt_Iq;
  float *real_Iq = pow_lim->real_Iq;
  float *real_omega = pow_lim->real_omega;
  float *expt_omega = pow_lim->expt_omega;

  ///>>> 根据电机功率模型求解原始设定值所需要的功率
  float total_power = 0;
  for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    pow_lim->raw_power[i] =
        get_chasiss_motors_power(expt_omega[i], raw_expt_Iq[i]);
    total_power += pow_lim->raw_power[i];
  }
  /**按功率占比分配**/
#if ENABLE_POWER_LIMIT_FLAG == 1
  if (total_power > pow_lim->set_total_power) {
    power_limit_ratio = pow_lim->set_total_power / total_power;
  } else {
    power_limit_ratio = 1.0f;
  }
#endif
  /**根据功率逆向解算,得到在当前角速度下,为达到设定功率所需的设定电流**/
  float set_total_power = 0;
  for (uint8_t i = 0; i < CHASSIS_MOTOR_NUM; i++) {
    pow_lim->set_power[i] = pow_lim->raw_power[i] * power_limit_ratio;
    set_total_power += pow_lim->set_power[i];
    int dirc = sign(raw_expt_Iq[i]);
    pow_lim->real_power[i] =
        get_chasiss_motors_power(real_omega[i], real_Iq[i]);

    float set_Iq = get_chassis_motors_Iq( //
        pow_lim->set_power[i], real_omega[i], dirc);
    set_Iq = CLAMP(set_Iq, C620_CURR_MAX);
    if (dirc >= 0) {
      LIMIT_MIN_MAX(set_Iq, 0, raw_expt_Iq[i]);
    } else {
      LIMIT_MIN_MAX(set_Iq, raw_expt_Iq[i], 0);
    }
    pow_lim->set_Iq[i] = set_Iq;
  }
#endif
  /**设置超电最大输入功率**/
  static float last_set_total_power = 0;
  float d_set_total_power =
      (set_total_power - last_set_total_power) * 1000.f / delta_tick;
  if (d_set_total_power < 0)
    d_set_total_power = 0;
  last_set_total_power = set_total_power;

  set_charge_power = pow_lim->power_limit - k_feedforward * d_set_total_power;
  set_charge_power_filter.ts = delta_tick / 1000.f;
  set_charge_power = LPF_update(&set_charge_power_filter, set_charge_power);
  LIMIT_MIN_MAX(set_charge_power, 0.f, 120.f);
  set_cap_power(&SUPER_CAP_CAN_HANDLE, set_charge_power, cap_state);
}

/*void power_ctrl_old_init(chassis_power_lim_old_t* chassis_power_lim_old){
        memcpy(&self_power_base_ratio_pid,&self_power_base_ratio_pid_init_val,sizeof(pid_struct_t));

        memcpy(&self_power_limit_pid,&self_power_limit_pid_init_val,sizeof(pid_struct_t));
        memcpy(&self_power_unlimit_pid,&self_power_unlimit_pid_init_val,sizeof(pid_struct_t));

 //保持以下初始化值等同于不限制功率
        chassis_power_lim_old->max_total_out = 4.0f*C620_OUTPUT_MAX;
        chassis_power_lim_old->vxy_output_max = chassis_vx_pid_init_val.out_max;
        chassis_power_lim_old->wz_output_max = chassis_wz_pid_init_val.out_max;
        chassis_power_lim_old->vxy_ratio = 1.0f;
        chassis_power_lim_old->wz_ratio = 1.0f;
        chassis_power_lim_old->power_limit_ratio = 1.0f;
}*/

#if robot_type == poor_infantry
#define PMM_RATIO 0.85f // 底盘全功率时电流过大，会导致电源管理模块保护性断电
#endif
#if robot_type == omni_infantry
#define PMM_RATIO 0.85f
#endif

/*void self_power_ctrl(chassis_power_lim_old_t*
chassis_power_lim)//输出chassis_power_lim
{

        float result_ratio[2], power_base_ratio;

        chassis_power_lim->max_total_out = 4.0f*C620_OUTPUT_MAX;

        power_base_ratio = pid_calc(&self_power_base_ratio_pid,
game_robot_status.chassis_power_limit * 1.2f, power_heat_data.chassis_power);
        LIMIT_MIN_MAX(power_base_ratio,0.f,1.f);

  result_ratio[0] = pid_calc(&self_power_limit_pid,
power_heat_data.chassis_power_buffer, POWER_BUFFER_PEAK * 0.9f) *
power_base_ratio; result_ratio[1] = pid_calc(&self_power_unlimit_pid,
power_heat_data.chassis_power_buffer, POWER_BUFFER_PEAK * 0.2f) *
power_base_ratio; uint8_t use_buffer = 0; if(result_ratio[use_buffer] < 0)
        {result_ratio[use_buffer] = 0.f;}
        else if(game_robot_status.chassis_power_limit ==
65535){result_ratio[use_buffer] = PMM_RATIO;}
        chassis_power_lim->power_limit_ratio = result_ratio[use_buffer];
}*/
