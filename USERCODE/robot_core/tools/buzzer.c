#include "buzzer.h"
#include HAL_INCLUDE

extern TIM_HandleTypeDef BEEP_TIM_HANDLE;

#define MAX_BEEP_KICK_CD_CNT 400 // 10HZ计数 则4秒需要计数400次

volatile uint32_t beep_cycle = 0, beep_cnt = 0, beep_cnt_loaded = 0;
volatile uint16_t beep_kick_cd_cnt = 0;

#define BEEP_KICK_BIT_NUM 5
static volatile bool beep_kick_bit[BEEP_KICK_BIT_NUM] = {0};

void beep(uint32_t cycle, uint32_t cnt, uint32_t ARR) {
  BEEP_TIM_HANDLE.Instance->ARR = ARR;
  beep_cycle = 2 * cycle;
  // beep_cycle = cycle;
  beep_cnt_loaded = cnt;
  beep_cnt = 0;
}

void beep_kick_start(struct_beep_t index) { beep_kick_bit[index] = true; }

void beep_kick_stop(struct_beep_t index) { beep_kick_bit[index] = false; }

bool beep_start_show_enable = false;

void buzzer_proc(void) {
  uint32_t beep_psc = ((beep_cycle + 1) % 2) * 125;
  // beep_psc = beep_cycle *125;
  if (beep_cycle > 0) {
    if (beep_cnt > 0)
      beep_cnt--;
    else {
      __HAL_TIM_SetCompare(&BEEP_TIM_HANDLE, BEEP_TIM_CHANNAL, beep_psc);
      beep_cnt = beep_cnt_loaded;
      beep_cycle--;
    }
  }
  // else __HAL_TIM_SetCompare(&BEEP_TIM_HANDLE, BEEP_TIM_CHANNAL, 0);
  beep_kick_cd_cnt++;
  bool beep_kick_request_flag = false;
  for (uint8_t i = 0; i < BEEP_KICK_BIT_NUM; i++) {
    if (beep_kick_bit[i]) {
      if (beep_kick_cd_cnt > MAX_BEEP_KICK_CD_CNT) {
        beep_kick_cd_cnt = 0;
        beep(1, 1, 250);
      }
      beep_kick_request_flag = true;
      break;
    }
  }
  if (!beep_kick_request_flag) {
    beep_kick_cd_cnt = 0;
  }
}
void buzzer_on(uint32_t n, uint32_t ARR) {
  BEEP_TIM_HANDLE.Instance->ARR = ARR;
  __HAL_TIM_SetCompare(&BEEP_TIM_HANDLE, BEEP_TIM_CHANNAL, n);
}
