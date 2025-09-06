
#include "pid_fuzzy.h"

#include "string.h"
#include <stdint.h>

#ifndef INVERSE
#define INVERSE(parameter) 1.0f / (float)parameter
#endif // !INVERSE

// Gaussian membership function
static float gaussmf(float x, float sigma, float c) {
  return expf(-powf(((x - c) / sigma), 2.0f));
}

// Generalized bell-shaped membership function
static float gbellmf(float x, float a, float b, float c) {
  return INVERSE(1.0f + powf(fabsf((x - c) / a), 2.0f * b));
}

// Sigmoidal membership function
static float sigmf(float x, float a, float c) {
  return INVERSE(1.0f + expf(a * (c - x)));
}

// Trapezoidal membership function
static float trapmf(float x, float a, float b, float c, float d) {
  if (x >= a && x < b)
    return (x - a) / (b - a);
  else if (x >= b && x < c)
    return 1.0f;
  else if (x >= c && x <= d)
    return (d - x) / (d - c);
  else
    return 0.0f;
}

// Triangular membership function
static float trimf(float x, float a, float b, float c) {
  return trapmf(x, a, b, b, c);
}

// Z-shaped membership function
static float zmf(float x, float a, float b) {
  if (x <= a)
    return 1.0f;
  else if (x >= a && x <= (a + b) / 2.0f)
    return 1.0f - 2.0f * powf((x - a) / (b - a), 2.0f);
  else if (x >= (a + b) / 2.0f && x < b)
    return 2.0f * powf((x - b) / (b - a), 2.0f);
  else
    return 0;
}

// Membership function
static float mf(float x, uint32_t mf_type, int32_t *params) {
  switch (mf_type) {
  case 0:
    return gaussmf(x, params[0], params[1]);
  case 1:
    return gbellmf(x, params[0], params[1], params[2]);
  case 2:
    return sigmf(x, params[0], params[2]);
  case 3:
    return trapmf(x, params[0], params[1], params[2], params[3]);
  case 5:
    return zmf(x, params[0], params[1]);
  default: // set triangular as default membership function
    return trimf(x, params[0], params[1], params[2]);
  }
}

// Union operator
static float or (float a, float b, uint32_t type) {
  if (type == 1) { // algebraic sum
    return a + b - a * b;
  } else if (type == 2) { // bounded sum
    return fminf(1, a + b);
  } else { // fuzzy union
    return fmaxf(a, b);
  }
}

// Intersection operator
static float and (float a, float b, uint32_t type) {
  if (type == 1) { // algebraic product
    return a * b;
  } else if (type == 2) { // bounded product
    return fmaxf(0, a + b - 1);
  } else { // fuzzy intersection
    return fminf(a, b);
  }
}

// Equilibrium operator
static float equilibrium(float a, float b, float params) {
  return powf(a * b, 1 - params) * powf(1 - (1 - a) * (1 - b), params);
}

// Fuzzy operator
static float fo(float a, float b, uint32_t type) {
  if (type < 3) {
    return and(a, b, type);
  } else if (type < 6) {
    return or (a, b, type - 3);
  } else {
    return equilibrium(a, b, 0.5f);
  }
}

// Mean of centers defuzzifier, only for two input multiple index
static void moc(const float *joint_membership, const uint32_t *index,
                const uint32_t *count, fuzzy_t *fuzzy_struct) {

  float denominator_count = 0;
  float numerator_count[fuzzy_struct->output_num];
  for (uint32_t l = 0; l < fuzzy_struct->output_num; ++l) {
    numerator_count[l] = 0;
  }

  for (int32_t i = 0; i < count[0]; ++i) {
    for (int32_t j = 0; j < count[1]; ++j) {
      denominator_count += joint_membership[i * count[1] + j];
    }
  }

  for (uint32_t k = 0; k < fuzzy_struct->output_num; ++k) {
    for (uint32_t i = 0; i < count[0]; ++i) {
      for (uint32_t j = 0; j < count[1]; ++j) {
        numerator_count[k] +=
            joint_membership[i * count[1] + j] *
            fuzzy_struct
                ->rule_base[k * QF_DEFAULT * QF_DEFAULT +
                            index[i] * QF_DEFAULT + index[count[0] + j]];
      }
    }
  }
}

// Defuzzifier
static void df(const float *joint_membership, const uint32_t *output,
               const uint32_t *count, fuzzy_t *fuzzy_struct, int32_t df_type) {
  if (df_type == 0)
    moc(joint_membership, output, count, fuzzy_struct);
  else {
    // printf("Waring: No such of defuzzifier!\n");
    moc(joint_membership, output, count, fuzzy_struct);
  }
}

static void fuzzy_process(float err, float err_dot, fuzzy_t *fuzzy) {
  float membership[QF_DEFAULT * 2]; // Membership values for storage errors and
                                    // error change rates
  uint32_t index[QF_DEFAULT *
                 2]; // Store rule indexes corresponding to membership degrees
  uint32_t count[2] = {0,
                       0}; // Number of effective membership degrees for storing
                           // errors and error change rates separately

  // Fuzzification
  {
    uint8_t j = 0;
    for (uint8_t i = 0; i < QF_DEFAULT; ++i) {
      float temp = mf(err, fuzzy->mf_type[0], fuzzy->mf_params + 4 * i);
      if (temp > 1e-4f) {
        membership[j] = temp;
        index[j++] = i;
      }
    }
    count[0] = j;
    for (int32_t i = 0; i < QF_DEFAULT; ++i) {
      float temp = mf(err_dot, fuzzy->mf_type[1], fuzzy->mf_params + 4 * i);
      if (temp > 1e-4f) {
        membership[j] = temp;
        index[j++] = i;
      }
    }
    count[1] = j - count[0];
  }

  // Check the effective membership degree
  if (count[0] == 0 || count[1] == 0) {
    for (uint32_t l = 0; l < fuzzy->output_num; ++l) {
      fuzzy->output[l] = 0;
    }
    return;
  }

  // Joint membership
  float joint_membership[count[0] * count[1]];

  for (int32_t i = 0; i < count[0]; ++i) {
    for (int32_t j = 0; j < count[1]; ++j) {
      joint_membership[i * count[1] + j] =
          fo(membership[i], membership[count[0] + j], fuzzy->fo_type);
    }
  }

  // Defuzzification
  df(joint_membership, index, count, fuzzy, 0);
}

/**
 * @brief Initialize fuzzy controller
 * @param fuzzy: fuzzy controller structure
 * @param output_num: number of output, 1->kp, 2->kp and ki, 3->kp, ki and kd
 */
void fuzzy_triangle_init(fuzzy_t *fuzzy, uint32_t output_num) {
  fuzzy->input_num = 2;
  fuzzy->output_num = output_num;
  for (uint32_t i = 0; i < fuzzy->input_num + fuzzy->output_num; ++i) {
    fuzzy->mf_type[i] = 4;
  }

  for (uint32_t i = 0; i < fuzzy->output_num; ++i) {
    fuzzy->output[i] = 0;
  }

  static int32_t mf_[4 * QF_DEFAULT] = {-3, -3, -2, 0, -3, -2, -1, 0, -2, -1,
                                        0,  0,  -1, 0, 1,  0,  0,  1, 2,  0,
                                        1,  2,  3,  0, 2,  3,  3,  0};
  fuzzy->mf_params = &mf_[0];
  static int32_t _rule[][QF_DEFAULT] = {// delta kp rule base
                                        {PB, PB, PM, PM, PS, ZO, ZO},
                                        {PB, PB, PM, PS, PS, ZO, NS},
                                        {PM, PM, PM, PS, ZO, NS, NS},
                                        {PM, PM, PS, ZO, NS, NM, NM},
                                        {PS, PS, ZO, NS, NS, NM, NM},
                                        {PS, ZO, NS, NM, NM, NM, NB},
                                        {ZO, ZO, NM, NM, NM, NB, NB},
                                        // delta ki rule base
                                        {NB, NB, NM, NM, NS, ZO, ZO},
                                        {NB, NB, NM, NS, NS, ZO, ZO},
                                        {NB, NM, NS, NS, ZO, PS, PS},
                                        {NM, NM, NS, ZO, PS, PM, PM},
                                        {NM, NS, ZO, PS, PS, PM, PB},
                                        {ZO, ZO, PS, PS, PM, PB, PB},
                                        {ZO, ZO, PS, PM, PM, PB, PB},
                                        // delta kd rule base
                                        {PS, NS, NB, NB, NB, NM, PS},
                                        {PS, NS, NB, NM, NM, NS, ZO},
                                        {ZO, NS, NM, NM, NS, NS, ZO},
                                        {ZO, NS, NS, NS, NS, NS, ZO},
                                        {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
                                        {PB, PS, PS, PS, PS, PS, PB},
                                        {PB, PM, PM, PM, PS, PS, PB}};
  fuzzy->rule_base = &_rule[0][0];
  fuzzy->fo_type = 1;
  fuzzy->df_type = 0;
}

float pid_fuzzy_calc(float exp, float cur, float err_max, float derr_max,
                     pid_struct_t *pid, fuzzy_t *fuzzy) {
  pid->err[1] = pid->err[0];
  pid->err[0] = exp - cur;

  float err_dot = pid->err[0] - pid->err[1];
  fuzzy_process(pid->err[0] / err_max * 3.f, err_dot / derr_max * 3.f, fuzzy);
  fuzzy->d_kp = fuzzy->output[0] / 3.f * fuzzy->d_kp_max;
  if (fuzzy->output_num >= 2)
    fuzzy->d_ki = fuzzy->output[1] / 3.0f * fuzzy->d_ki_max;
  else
    fuzzy->d_ki = 0;

  if (fuzzy->output_num >= 3)
    fuzzy->d_kd = fuzzy->output[2] / 3.0f * fuzzy->d_kd_max;
  else
    fuzzy->d_kd = 0;
  pid->p_out = (pid->kp + fuzzy->d_kp) * pid->err[0];
  pid->i_out += (pid->ki + fuzzy->d_ki) * pid->err[0] * pid->dt;
  if (isnan(pid->i_out))
    pid->i_out = 0;
  pid->i_out = pid->i_out > pid->i_max
                   ? pid->i_max
                   : (pid->i_out < -pid->i_max ? -pid->i_max
                                               : pid->i_out); // Anti-windup
  pid->d_out = (pid->kd + fuzzy->d_kd) * err_dot / pid->dt;
  pid->out = pid->p_out + pid->i_out + pid->d_out;
  pid->out = pid->out > pid->out_max
                 ? pid->out_max
                 : (pid->out < -pid->out_max ? -pid->out_max : pid->out);
  return pid->out;
}
float pid_fuzzy_calc_deadband(float exp, float cur, float err_max,
                              float derr_max, pid_struct_t *pid,
                              fuzzy_t *fuzzy) {
  pid->err[1] = pid->err[0];
  pid->err[0] = exp - cur;

  if (pid->err[0] > pid->k_deadband) {
    pid->err[0] -= pid->k_deadband;
  } else if (pid->err[0] < -pid->k_deadband) {
    pid->err[0] += pid->k_deadband;
  } else {
    if (exp < pid->k_deadband &&
        exp > -pid->k_deadband // reference value get into deadband
        && (pid->err[1] < -pid->k_deadband ||
            pid->err[1] > pid->k_deadband)) { // last error out of deadband
      pid->i_out = 0.0f;
    }
    if (pid->deadband_zero_output)
      return 0.0f;
  }

  float err_dot = pid->err[0] - pid->err[1];
  fuzzy_process(pid->err[0] / err_max * 3.f, err_dot / derr_max * 3.f, fuzzy);
  fuzzy->d_kp = fuzzy->output[0] / 3.f * fuzzy->d_kp_max;
  if (fuzzy->output_num >= 2)
    fuzzy->d_ki = fuzzy->output[1] / 3.0f * fuzzy->d_ki_max;
  else
    fuzzy->d_ki = 0;

  if (fuzzy->output_num >= 3)
    fuzzy->d_kd = fuzzy->output[2] / 3.0f * fuzzy->d_kd_max;
  else
    fuzzy->d_kd = 0;
  pid->p_out = (pid->kp + fuzzy->d_kp) * pid->err[0];
  pid->i_out += (pid->ki + fuzzy->d_ki) * pid->err[0] * pid->dt;
  if (isnan(pid->i_out))
    pid->i_out = 0;
  pid->i_out = pid->i_out > pid->i_max
                   ? pid->i_max
                   : (pid->i_out < -pid->i_max ? -pid->i_max
                                               : pid->i_out); // Anti-windup
  pid->d_out = (pid->kd + fuzzy->d_kd) * err_dot / pid->dt;
  pid->out = pid->p_out + pid->i_out + pid->d_out;
  pid->out = pid->out > pid->out_max
                 ? pid->out_max
                 : (pid->out < -pid->out_max ? -pid->out_max : pid->out);
  return pid->out;
}

float pid_fuzzy_dualloop(pid_struct_t pid[2], fuzzy_t *fuzzy, float err,
                         float err_max, float derr_max, float err_out,
                         float err_out_max, float derr_out_max) {
  float pid_in_output;
  pid_in_output = pid_fuzzy_calc(0, err, err_max, derr_max, &pid[0], fuzzy);
  return pid_fuzzy_calc(pid_in_output, err_out, err_out_max, derr_out_max,
                        &pid[1], fuzzy);
}
