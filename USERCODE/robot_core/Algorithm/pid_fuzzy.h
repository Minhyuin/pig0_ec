#ifndef PID_FUZZY_H_
#define PID_FUZZY_H_

#include "pid.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "./pid.h"
#include "stdint.h"

// Fuzzy quantity fields
enum quantity_fields {
  QF_SMALL = 5,
  QF_MIDDLE = 7,
  QF_LARGE = 8,
};

#define QF_DEFAULT QF_MIDDLE

typedef struct _fuzzy {
  uint32_t input_num;
  uint32_t output_num;
  uint32_t fo_type;    // fuzzy method
  uint32_t mf_type[5]; // membership function type
  int32_t *mf_params;  // membership function parameters
  uint32_t df_type;    // defuzzification method
  int32_t *rule_base;  // rule base

  float d_kp;
  float d_ki;
  float d_kd;

  float d_kp_max;
  float d_ki_max;
  float d_kd_max;

  float output[3];
} fuzzy_t;

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

void fuzzy_triangle_init(fuzzy_t *fuzzy, uint32_t output_num);
float pid_fuzzy_calc(float exp, float cur, float err_max, float derr_max,
                     pid_struct_t *pid, fuzzy_t *fuzzy);
float pid_fuzzy_calc_deadband(float exp, float cur, float err_max,
                              float derr_max, pid_struct_t *pid,
                              fuzzy_t *fuzzy);

float pid_fuzzy_dualloop(pid_struct_t pid[2], fuzzy_t *fuzzy, float err,
                         float err_max, float derr_max, float err_out,
                         float err_out_max, float derr_out_max);

#ifdef __cplusplus
}
#endif

#endif // !PIF_FUZZY_H_
