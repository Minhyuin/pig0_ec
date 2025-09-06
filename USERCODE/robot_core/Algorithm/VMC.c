/**
 * @file VMC.c
 * @brief 针对轮腿机器人的VMC五连杆解算
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 */
#include "VMC.h"
#include "../drv_conf.h"
#include "util.h"

#include "../Base/base_chassis.h"

// static float L_hl = 0, L_hr = 0;
static float L_1 = 0, L_2 = 0, L_3 = 0, L_4 = 0, L_5 = 0;
/**
 * @note 视图
 *  VMC建图坐标系:
 *  ↑ y
 *  |             5 C
 *  ·——-> x    L2  /·\  L3
 *                /   \
 *           B 2 /     \ 3 D
 *           L1  \_____/  L4
 *           A 1   L5   4 E
 */

/**
 * @brief 机器人参数初始化
 *
 */
void robot_mechanical_parameters_init(void) {
  L_1 = 0.150f; // 大腿
  L_4 = L_1;    // 大腿
  L_2 = 0.250f; // 小腿
  L_3 = L_2;    // 小腿
  L_5 = 0.120f; // 关节电机间距
}

dual_vec_t L_cstr = {0.1f, 0.3f};
/**
 * @brief 获取腿在含机械限位情况下的最大跟最小腿长
 *
 * @return dual_vec_t* L_c={L_min,L_max}
 */
dual_vec_t *get_L0_constrict(void) {
#if defined(hip_sit_cstr_theta)
  dual_vec_t phi2_and_phi_3_min = *get_knee_phi(PHI1_MIN, PHI4_MIN);
  dual_vec_t phi2_and_phi_3_max = *get_knee_phi(PHI1_MAX, PHI4_MAX);
  float phi2_min = phi2_and_phi_3_min._11;
  float phi2_max = phi2_and_phi_3_max._11;

  dual_vec_t phi0_and_L0_min = *get_real_phi0_and_L0(PHI1_MIN, phi2_min);
  dual_vec_t phi0_and_L0_max = *get_real_phi0_and_L0(PHI1_MAX, phi2_max);

  L_cstr._11 = phi0_and_L0_min._21; // MIN
  L_cstr._21 = phi0_and_L0_max._21; // MAX
#endif

  return &L_cstr;
}
// 腿部VMC
/**
 * @brief 获取单侧膝关节两角度值(即phi2与phi3)
 * @param phi_1
 * @param phi_4
 * @return dual_vec_t* phi2_and_3
 */
dual_vec_t *get_knee_phi(float phi_1, float phi_4) {
  static dual_vec_t phi2_and_3 = {0};

  // 计算当前腿的姿态(phi_2 phi_3)
  float x_A, y_A, x_B, y_B, x_D, y_D, x_E, y_E;
  x_A = -L_5 / 2;
  y_A = 0;
  x_E = L_5 / 2;
  y_E = 0;
  x_B = x_A + L_1 * cosf(phi_1);
  y_B = y_A + L_1 * sinf(phi_1);
  x_D = x_E + L_4 * cosf(phi_4);
  y_D = y_E + L_4 * sinf(phi_4);

  float x_BD, y_BD, L_BD;
  x_BD = x_B - x_D;
  y_BD = y_B - y_D;
  L_BD = 1.f / fast_inv_sqrt(pow2f(x_BD) + pow2f(y_BD));

  float k = 0, abs_theta = 0, add_theta = 0;
  k = fabsf((pow2f(L_3) - (pow2f(L_2) + pow2f(L_BD))) / (2 * L_2 * L_BD));
  abs_theta = acosf(k);
  // 等腰三角形角度
  add_theta = atanf(y_BD / x_BD); // 不能用atan2f
  // 等腰三角形底线的倾斜角度(斜率)
  // 结果的定义域 : (-pi / 2, pi / 2)

  float phi_2, phi_3;
  phi_2 = abs_theta + add_theta;
  phi_3 = PI - (abs_theta - add_theta);

  phi2_and_3 = (dual_vec_t){._11 = phi_2, ._21 = phi_3};
  return &phi2_and_3;
}

/**
 * @brief 获得单侧五连杆末端对应极坐标的角度与摆杆长度
 * @param phi_1
 * @param phi_2
 * @return dual_vec_t* phi0_and_L0
 */
dual_vec_t *get_real_phi0_and_L0(float phi_1, float phi_2) {
  static dual_vec_t phi0_and_L0 = {0};

  float x_C = L_2 * cosf(phi_2) + L_1 * cosf(phi_1) - L_5 / 2;
  float y_C = L_2 * sinf(phi_2) + L_1 * sinf(phi_1);

  // float phi_C = atan2f(y_C, x_C);
  // float phi_0 = -(PI / 2 - phi_C);
  float phi_0 = atan2f(y_C, x_C) - PI / 2;
  float L_0 = 1.f / fast_inv_sqrt(pow2f(y_C) + pow2f(x_C));

  phi0_and_L0 = (dual_vec_t){._11 = phi_0, ._21 = L_0};
  return &phi0_and_L0;
}

/**
 * @brief 单侧五连杆机构逆向解算,获得J矩阵
 *        可根据J_T计算出 五连杆末端虚拟力Fx,Fy 与 关节扭矩T1,T4 之间的关系
 * @param phi_1
 * @param phi_2
 * @param phi_3
 * @param phi_4
 * @return quad_vec_t* J
 * @note The definition of angle refers to
 *        https://zhuanlan.zhihu.com/p/613007726
 */
quad_vec_t *TF_Cartesian2FiveRod(float phi_1, float phi_2, float phi_3,
                                 float phi_4) {
  static quad_vec_t J = {0}; // 描述关节扭矩与驱动轮端的虚拟力间的关系

  float sinf12 = sinf(phi_1 - phi_2);
  float sinf34 = sinf(phi_3 - phi_4);
  float sinf23 = sinf(phi_2 - phi_3);

  J = (quad_vec_t){
      ._11 = +L_1 * sinf(phi_3) * sinf12 / sinf23,
      ._12 = +L_4 * sinf(phi_2) * sinf34 / sinf23,
      ._21 = -L_1 * cosf(phi_3) * sinf12 / sinf23,
      ._22 = -L_4 * cosf(phi_2) * sinf34 / sinf23,
  };
  return &J;
}

/**
 * @brief 获得 笛卡尔坐标系 转 极坐标系 的矩阵R
 * @param phi_1
 * @param phi_2
 * @return quad_vec_t* R
 */
quad_vec_t *TF_Polar2Cartesian(float phi_1, float phi_2) {
  static quad_vec_t R = {0};

  float x_C = L_2 * cosf(phi_2) + L_1 * cosf(phi_1) - L_5 / 2;
  float y_C = L_2 * sinf(phi_2) + L_1 * sinf(phi_1);

  float phi_C = atan2f(y_C, x_C);
  float K = pow2f(x_C) * fast_inv_sqrt(pow2f(y_C) + pow2f(x_C));
  // float L_0 = 1.f / fast_inv_sqrt(pow2f(y_C) + pow2f(x_C));
  // float K = pow2f(x_C) / L_0;

  R = (quad_vec_t){
      ._11 = -sinf(phi_C),
      ._12 = +cosf(phi_C),
      ._21 = +cosf(phi_C),
      ._22 = +sinf(phi_C),
  };

  return &R;
}

/**
 * @brief 获得 切向力Fq 变为 切向转矩Tq 的矩阵
 * @param L_0
 * @return quad_vec_t*
 */
quad_vec_t *TF_ForceQ2TorqueQ(float L_0) {
  static quad_vec_t M = {0};
  M = (quad_vec_t){._11 = 1 / L_0, ._12 = 0, ._21 = 0, ._22 = 1};
  return &M;
}

/**
 * @brief 单侧极坐标控制器变换器
 *        可将设定的 切向力矩T_q与径向力F_d 变换为 输出给两髋关节电机的力矩T1,T4
 * @param expt_Tq 设定切向力矩
 * @param expt_Fd 设定径向力
 * @return dual_vec_t* T1_and_T4
 */
dual_vec_t *TF_FiveRod2Polar(float expt_Tq, float expt_Fd, quad_vec_t *J,
                             quad_vec_t *R, quad_vec_t *M) {
  dual_vec_t Fq_and_Fd = {0}, Fx_and_Fy = {0};
  static dual_vec_t T1_and_T4 = {0};

  quad_vec_t J_T = {._11 = J->_11, ._12 = J->_21, ._21 = J->_12, ._22 = J->_22};

  Fq_and_Fd = (dual_vec_t){
      ._11 = M->_11 * expt_Tq + M->_12 * expt_Fd, // Fq
      ._21 = M->_21 * expt_Tq + M->_22 * expt_Fd, // Fd
  };
  Fx_and_Fy = (dual_vec_t){
      ._11 = R->_11 * Fq_and_Fd._11 + R->_12 * Fq_and_Fd._21, // Fx
      ._21 = R->_21 * Fq_and_Fd._11 + R->_22 * Fq_and_Fd._21, // Fy
  };
  T1_and_T4 = (dual_vec_t){
      ._11 = J_T._11 * Fx_and_Fy._11 + J_T._12 * Fx_and_Fy._21, // torque1
      ._21 = J_T._21 * Fx_and_Fy._11 + J_T._22 * Fx_and_Fy._21, // torque4
  };

  return &T1_and_T4;
}

/**
 * @brief 根据 髋关节电机实际输出扭矩T1,T4 逆向求解出 输出实际有效输出扭矩Tq,Fd
 *        (避免单关节过载导致实际值与设定值之间非线性)
 * @param real_T1 控制phi1角的关节电机的实际力矩
 * @param real_T4 控制phi4角的关节电机的实际力矩
 * @return dual_vec_t* Tq_and_Fd
 */
dual_vec_t *get_valid_Tqd(float real_T1, float real_T4, quad_vec_t *J,
                          quad_vec_t *R, quad_vec_t *M) {
  dual_vec_t Fq_and_Fd = {0}, Fx_and_Fy = {0};
  static dual_vec_t Tq_and_Fd = {0};

  quad_vec_t J_T = {._11 = J->_11, ._12 = J->_21, ._21 = J->_12, ._22 = J->_22};

  quad_vec_t J_T_inv = get_quad_vec_inv(&J_T), R_inv = get_quad_vec_inv(R),
             M_inv = get_quad_vec_inv(M);

  Fx_and_Fy = (dual_vec_t){
      ._11 = J_T_inv._11 * real_T1 + J_T_inv._12 * real_T4,
      ._21 = J_T_inv._21 * real_T1 + J_T_inv._22 * real_T4,
  };
  Fq_and_Fd = (dual_vec_t){
      ._11 = R_inv._11 * Fx_and_Fy._11 + R_inv._12 * Fx_and_Fy._21,
      ._21 = R_inv._21 * Fx_and_Fy._11 + R_inv._22 * Fx_and_Fy._21,
  };
  Tq_and_Fd = (dual_vec_t){
      ._11 = M_inv._11 * Fq_and_Fd._11 + M_inv._12 * Fq_and_Fd._21,
      ._21 = M_inv._21 * Fq_and_Fd._11 + M_inv._22 * Fq_and_Fd._21,
  };

  return &Tq_and_Fd;
}
/**
 * @brief 单侧极坐标控制器位置解算器
 *        根据两髋关节电机角度解算出五连杆末端的速度
 * @param omega_1
 * @param omega_4
 * @param J
 * @param R
 * @return dual_vec_t*
 */
dual_vec_t *get_real_omega_and_v(float omega_1, float omega_4, quad_vec_t *J,
                                 quad_vec_t *R) {
  dual_vec_t real_vxy = {0};
  static dual_vec_t real_vqd = {0};

  real_vxy = (dual_vec_t){
      ._11 = J->_11 * omega_1 + J->_12 * omega_4, // vx
      ._21 = J->_21 * omega_1 + J->_22 * omega_4, // vy
  };
  real_vqd = (dual_vec_t){
      ._11 = R->_11 * real_vxy._11 + R->_12 * real_vxy._21, // vq
      ._21 = R->_21 * real_vxy._11 + R->_22 * real_vxy._21, // vd
  };

  return &real_vqd;
}
