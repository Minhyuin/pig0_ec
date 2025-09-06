#ifndef _VMC_H_
#define _VMC_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "../drv_conf.h"
#include "matrix.h"
#include HAL_INCLUDE

#define PHI1_MIN (deg2rad(180 - hip_sit_cstr_theta))
#define PHI4_MIN (deg2rad(360 + hip_sit_cstr_theta))
#define PHI1_MAX (deg2rad(180 - hip_std_cstr_theta))
#define PHI4_MAX (deg2rad(360 + hip_std_cstr_theta))

#ifndef DEFINED_get_quad_vec_inv
#define DEFINED_get_quad_vec_inv
/**
 * @brief Get the quad vec inv object
 *
 * @param M the matrix that requires inversion
 * @return quad_vec_t
 * @ref https://zhuanlan.zhihu.com/p/558667088
 */
static quad_vec_t get_quad_vec_inv(quad_vec_t *M) {
  float det_M = M->_11 * M->_22 - M->_12 * M->_21;
  // quad_vec_t M_inv = {
  //     ._11 = M->_22 / det_M,
  //     ._22 = M->_11 / det_M,
  //     ._12 = -M->_12 / det_M,
  //     ._21 = -M->_21 / det_M,
  // };
  quad_vec_t M_inv = {
      ._11 = M->_22 / det_M,
      ._12 = -M->_12 / det_M,
      ._21 = -M->_21 / det_M,
      ._22 = M->_11 / det_M,
  };

  return M_inv;
}
#endif

// pos
dual_vec_t *get_L0_constrict(void);
void robot_mechanical_parameters_init(void);
dual_vec_t *get_knee_phi(float phi_1, float phi_4);
dual_vec_t *get_real_phi0_and_L0(float phi_1, float phi_2);
// matrix
quad_vec_t *TF_Cartesian2FiveRod(float phi_1, float phi_2, float phi_3,
                                 float phi_4);
quad_vec_t *TF_Polar2Cartesian(float phi_1, float phi_2);
quad_vec_t *TF_ForceQ2TorqueQ(float L_0);
dual_vec_t *TF_FiveRod2Polar(float expt_Tq, float expt_Fd, quad_vec_t *J,
                             quad_vec_t *R, quad_vec_t *M);
dual_vec_t *get_valid_Tqd(float real_T1, float real_T4, quad_vec_t *J,
                          quad_vec_t *R, quad_vec_t *M);
dual_vec_t *get_real_omega_and_v(float omega_1, float omega_4, quad_vec_t *J,
                                 quad_vec_t *R);
#ifdef __cplusplus
}
#endif
#endif
