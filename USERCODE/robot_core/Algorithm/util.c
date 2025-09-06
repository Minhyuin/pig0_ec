#include "util.h"

/**
 * @brief  非阻塞式间隔指定时间执行一个函数
 * @param  func: 被执行函数(可以是赋值等其他语句)
 * @param  time: 设定间隔时间(ms)
 * @param  staticval: 上一次执行函数的时间(静态变量)
 * @param  millis_func: 获取当前时间的函数指针
 * @example stm32平台 使用 &HAL_GetTick 作为函数指针进入该函数
 * @retval 无
 */
void IntervalExecute(void (*func)(void), uint32_t time, uint32_t *staticval,
                     uint32_t (*millis_func)(void)) {
  uint32_t current_time = millis_func();

  if (current_time - *staticval >= time) {
    func();
    *staticval = current_time;
  }
}

/**
 * @brief  将一个函数重复调用指定次数
 * @param  func: 被调用函数
 * @param  n: 重复调用次数
 * @retval 无
 */
void LoopExecute(void (*func)(void), uint32_t n) {
  for (uint32_t i = 0; i < n; i++) {
    func(); // 调用传入的函数
  }
}
