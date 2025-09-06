#ifndef _LED_H
#define _LED_H
#ifdef __cplusplus
extern "C" {
#endif
#include "drv_conf.h"
#include HAL_INCLUDE

#if defined(STM32H723xx)
void RGB_loop(uint32_t tim_freq, float s, uint8_t mode);
#elif defined(STM32F407xx)
void RGB_loop(uint32_t tim_freq, float s);
#endif
#ifdef __cplusplus
}
#endif
#endif // !_LED_H
