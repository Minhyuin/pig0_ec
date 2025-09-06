#ifndef _BUZZER_H
#define _BUZZER_H

#include "drv_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  CHASSIS_MOTOR_OFFLINE_BEEP = 0U,
  GIMBAL_MOTOR_OFFLINE_BEEP,
  SHOOTER_MOTOR_OFFLINE_BEEP,
} struct_beep_t;

void buzzer_proc(void);
void beep(uint32_t cycle, uint32_t cnt, uint32_t ARR);
void beep_kick_start(struct_beep_t index);
void beep_kick_stop(struct_beep_t index);
void buzzer_on(uint32_t n, uint32_t ARR);

#ifdef __cplusplus
}
#endif

#endif
