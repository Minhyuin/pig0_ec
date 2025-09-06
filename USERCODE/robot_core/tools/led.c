#include "led.h"

#include "main.h"
#include "spi.h"

float real_s = 0;
uint8_t pin_index = 0;
GPIO_PinState pinstate[3] = {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET};
uint8_t pinstate_256x[3] = {0, 0, 0};
#if defined(STM32H723xx)
#define WS2812_LowLevel 0xC0  // 0码
#define WS2812_HighLevel 0xF0 // 1码

static void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t txbuf[24];
  uint8_t res = 0;
  for (int i = 0; i < 8; i++) {
    txbuf[7 - i] =
        (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    txbuf[15 - i] =
        (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    txbuf[23 - i] =
        (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
  }
  HAL_SPI_Transmit(&LED_SPI_HANDEL, &res, 0, 0xFFFF);
  while (LED_SPI_HANDEL.State != HAL_SPI_STATE_READY)
    ;
  HAL_SPI_Transmit(&LED_SPI_HANDEL, txbuf, 24, 0xFFFF);
  for (int i = 0; i < 100; i++) {
    HAL_SPI_Transmit(&LED_SPI_HANDEL, &res, 1, 0xFFFF);
  }
}
void RGB_loop(uint32_t tim_freq, float s, uint8_t mode) {
  // freq为所在定时器频率
  // s表示几秒换颜色
  float degree = 0.1f;
  real_s = (float)tim_freq / 1.f;
  if (real_s > s) {
    if (pin_index++ == 3)
      pin_index = 0;
    for (uint8_t i = 0; i < 3; i++) {
      switch (mode) {
      case 1:
        if (i != pin_index)
          pinstate_256x[i] = 255 * degree;
        else
          pinstate_256x[pin_index] = 0;
        break;
      case 2:
        if (i != pin_index)
          pinstate_256x[i] = 0;
        else
          pinstate_256x[pin_index] = 255 * degree;
      default:
        break;
      }
    }
    real_s = 0;
  }
  WS2812_Ctrl((uint8_t)pinstate_256x[0], (uint8_t)pinstate_256x[1],
              (uint8_t)pinstate_256x[2]);
  // HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, pinstate[0]);
  // HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, pinstate[1]);
  // HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, pinstate[2]);
}
#elif defined(STM32F407xx)
void RGB_loop(uint32_t tim_freq, float s) {
  // freq为所在定时器频率
  // s表示几秒换颜色
  real_s = (float)tim_freq / 1.f;
  if (real_s > s) {
    if (pin_index++ == 3)
      pin_index = 0;
    for (uint8_t i = 0; i < 3; i++) {
      if (i != pin_index)
        pinstate[i] = GPIO_PIN_RESET;
      else
        pinstate[pin_index] = GPIO_PIN_SET;
    }
    real_s = 0;
  }
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, pinstate[0]);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, pinstate[1]);
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, pinstate[2]);
}
#endif
