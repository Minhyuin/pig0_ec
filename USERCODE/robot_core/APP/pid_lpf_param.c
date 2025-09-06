#include "pid_lpf_param.h"
#include "drv_conf.h"

// pid digital difference
/*PID参数文件*/
// chassis pid
//------------------------------------------------------------------------------------------
const pid_struct_t wheels_pid_init_val = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 0.01f,
    .i_max = C620_CURR_DATA_MAX / 50,
    .out_max = C620_CURR_DATA_MAX,
    .k_deadband = 500,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};
const pid_struct_t wheels_pid_isolated_init_val = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 0.01f,
    .i_max = C620_CURR_DATA_MAX / 50,
    .out_max = C620_CURR_DATA_MAX,
    .k_deadband = 500,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t chassis_vx_pid_init_val = {
    .kp = 1.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .i_max = 0.01f,
    .out_max = 0.1f,
    .k_deadband = 0.005f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t chassis_vy_pid_init_val = {
    .kp = 1.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .i_max = 0.01f,
    .out_max = 0.1f,
    .k_deadband = 0.005f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};
const pid_struct_t chassis_wz_pid_init_val = {
    .kp = 1.0f,
    .ki = 10.0f,
    .kd = 0.0f,
    .i_max = 0.01f,
    .out_max = 0.1f,
    .k_deadband = 0.001f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t chassis_follow_pid_init_val = {
    .kp = 0.0035f,
    .ki = 0.0f,
    .kd = 0.0005f,
    .i_max = 1,
    .out_max = 5, // 15,
    .k_deadband = 175.0f,
    .deadband_zero_output = 1,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};
const pid_struct_t chassis_vision_follow_pid_init_val = {
    .kp = 0.004f,
    .ki = 0.0f,
    .kd = 0.0005f,
    .i_max = 10,
    .out_max = 15,
    .k_deadband = 3000.0f,
    .deadband_zero_output = 1,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

// gimbal motor pid
//------------------------------------------------------------------------------------------

const pid_struct_t yaw_pid_ang_loop = {
    .kp = 0.1f,
    .ki = 0.0f,
    .kd = 0.001f,
    .i_max = GM6020_VOLT_MAX,
    .out_max = GM6020_VOLT_DATA_MAX / 10,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t yaw_pid_rpm_loop = {
    .kp = 0.9f,
    .ki = 0.00f,
    .kd = 0.0f,
    .i_max = GM6020_VOLT_MAX,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};

const pid_struct_t pitch_pid_ang_loop = {
    .kp = 0.12f,
    .ki = 0.02f,
    .kd = 0.0001f,
    .i_max = GM6020_VOLT_MAX,
    .out_max = GM6020_VOLT_DATA_MAX / 10,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t pitch_pid_rpm_loop = {
    .kp = 1.0f, // 200
    .ki = 0.0f, // 0.01
    .kd = 0.0f,
    .i_max = GM6020_VOLT_MAX,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};

// gimbal imu pid
//------------------------------------------------------------------------------------------
const pid_struct_t yaw_imu_pid_ang_vision_loop = {
    .kp = 0.25f,
    .ki = 0.8f,
    .kd = 0.0015f,
    .lin_range = 0.f,
    .i_max = 3.0f,
    .out_max = 2000,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t yaw_imu_pid_rpm_vision_loop = {
    .kp = 1.1f,
    .ki = 0.5f,
    .kd = 0.0f,
    .i_max = 1.5f,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t yaw_imu_pid_ang_loop = {
    .kp = 0.25f,
    .ki = 0.1f,
    .kd = 0.001f,
    .lin_range = 0.f,
    .i_max = 0.5f,
    .out_max = 200,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t yaw_imu_pid_rpm_loop = {
    .kp = 1.3f,
    .ki = 0.2f,
    .kd = 0.0f,
    .i_max = 5.f,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t pitch_imu_pid_ang_vision_loop = {
    .kp = 0.3f,
    .ki = 1.0f,
    .kd = 0.0f,
    .i_max = 4.f,
    .out_max = 2500,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t pitch_imu_pid_rpm_vision_loop = {
    .kp = 0.8f,
    .ki = 0.3f,
    .kd = 0.005f,
    .i_max = 0.8f,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t pitch_imu_pid_ang_loop = {
    .kp = 0.3f,
    .ki = 0.7f,
    .kd = 0.0005f,
    .i_max = 4.f,
    .out_max = GM6020_VOLT_DATA_MAX / 10,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
const pid_struct_t pitch_imu_pid_rpm_loop = {
    .kp = 0.8f,
    .ki = 0.2f,
    .kd = 0.0f,
    .i_max = 1.f,
    .out_max = GM6020_VOLT_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / GIMBAL_TIM_FREQ,
};
// shoot pid
//------------------------------------------------------------------------------------------
const pid_struct_t shoot_left_fric_pid_init_val = {
    .kp = 9.7f, // 15.0f,
    .ki = 0.0f,
    .kd = 0.0175f, // 12.0f,
    .i_max = C620_CURR_DATA_MAX / 50,
    .out_max = C620_CURR_DATA_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / SHOOTER_TIM_FREQ,
};

const pid_struct_t shoot_right_fric_pid_init_val = {
    .kp = 9.7f,    // 15.0f,
    .ki = 0.0f,    // 0.0f,
    .kd = 0.0175f, // 12.0f,
    .i_max = C620_CURR_DATA_MAX / 50,
    .out_max = C620_CURR_DATA_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / SHOOTER_TIM_FREQ,
};

const pid_struct_t dial_rpm_pid_init_val = {
    .kp = 0.007f,
    .ki = 0.0f,
    .kd = 0.0000005f,
    .i_max = 0.018f,
    .out_max = Kn_M2006 * C610_CURR_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / SHOOTER_TIM_FREQ,
};

const pid_struct_t dial_ang_pid_init_val = {
    .kp = 9.00f,
    .ki = 0.0001f,
    .kd = 0.25f,
    .i_max = 0.018f,
    .out_max = Kn_M2006 * C610_CURR_MAX,
    .k_deadband = 0.0f,
    .dt = 1.f / SHOOTER_TIM_FREQ,
};

// power limit pid
//------------------------------------------------------------------------------------------

const pid_struct_t self_power_base_ratio_pid_init_val = {
    .kp = 0.008f,
    .ki = 1.0f,
    .kd = 0.f,
    .i_max = 1.f,
    .out_max = 1.f,
    .k_deadband = 0.0f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t self_power_limit_pid_init_val = {
    .kp = 0.06f,
    .ki = 2000.f,
    .kd = 0.0f,
    .i_max = 0.1f,
    .out_max = 1.f,
    .k_deadband = 0.0f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t self_power_unlimit_pid_init_val = {
    .kp = 0.06f,
    .ki = 2000.f,
    .kd = 0.0f,
    .i_max = 0.1f,
    .out_max = 1.f,
    .k_deadband = 0.0f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t power_limit_init_val = {
    .kp = 0.002f,
    .ki = 1e-4f,
    .kd = 0.0f,
    .i_max = 0.1f,
    .out_max = 1.f,
    .k_deadband = 0.0f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t power_unlimit_init_val = {
    .kp = 0.0015f,
    .ki = 2000.f,
    .kd = 0.00000001f,
    .i_max = 0.1f,
    .out_max = 1.f,
    .k_deadband = 0.0f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const pid_struct_t power_buffer_r_contain_pid_init = {
    .kp = 0.001f,
    .ki = 0.0f,
    .kd = 0.0f,
    .i_max = 5,
    .out_max = 50,
    .k_deadband = 0.0f,
    .dt = 1.f / CHASSIS_TIM_FREQ,
};

const M_LPF_t pitch_lpf_init_val = {.fc = 0.00001f, .ts = H_TIM_FREQ};
const M_LPF_t yaw_lpf_init_val = {.fc = 0.00001f, .ts = H_TIM_FREQ};
const M_LPF_t R_fric_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f,
                                     .ts = M_TIM_FREQ};
const M_LPF_t L_fric_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f,
                                     .ts = M_TIM_FREQ};
const M_LPF_t dial_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f, .ts = M_TIM_FREQ};
const M_LPF_t RF_wheel_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f,
                                       .ts = M_TIM_FREQ};
const M_LPF_t LF_wheel_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f,
                                       .ts = M_TIM_FREQ};
const M_LPF_t LB_wheel_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f,
                                       .ts = M_TIM_FREQ};
const M_LPF_t RB_wheel_lpf_init_val = {.fc = M_TIM_FREQ / 2.56f,
                                       .ts = M_TIM_FREQ};
