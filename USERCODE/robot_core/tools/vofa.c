/**
 * @file vofa.c
 *
 * @brief 串口发送上位机(vofa)
 *
 *
 * @copyright SCNU-PIONEER (c) 2023-2024
 *
 */

#include "vofa.h"
#include "usbd_cdc_if.h"

#define MAX_FLOATS_NUM 10 // 最大浮点数数量
#define MAX_VOFA_BUF_LEN ((MAX_FLOATS_NUM * (4 + 3 + 1) + 1) + 1)
// 发送精度最大小数点前四位+小数点后两位

static char vofa_buffer[MAX_VOFA_BUF_LEN] = {0}; // 选择足够大的缓冲区
static uint16_t vofa_tx_data_len = 0;
struct {
  float MAX_TX_FREQ;
  float MIN_TX_TICK;
  float REAL_TX_FREQ;
} VOFA_INFO = {0};

static uint8_t FireWater_factory(float *fbuf) {
  char *buffer_ptr = vofa_buffer;
  uint8_t rslt = 0;
  vofa_tx_data_len = 0;
  for (char i = 0; i < MAX_FLOATS_NUM; i++) {
    vofa_tx_data_len +=
        snprintf(&buffer_ptr[vofa_tx_data_len],
                 sizeof(vofa_buffer) - vofa_tx_data_len, "%.3f,", *(fbuf + i));
  }
  // 添加 \r\n 并发送数据
  if (vofa_tx_data_len > 0) {
    vofa_buffer[vofa_tx_data_len - 2] = '\r';
    vofa_buffer[vofa_tx_data_len - 1] = '\n';
    rslt = 0;
  } else
    rslt = 1;
  return rslt;
}
void update_vofa_with_uart(UART_HandleTypeDef *huart, float *fbuf,
                           uint8_t float_nums) {
  if (float_nums > MAX_FLOATS_NUM)
    return;
  // 定义一个静态变量，用于记录上一次发送数据的时间
  static uint32_t tickstart = 0;
  // 如果串口状态为准备就绪，并且当前时间与上一次发送数据的时间间隔大于最小发送间隔+10ms
  if (huart->gState == HAL_UART_STATE_READY &&
      HAL_GetTick() - tickstart > VOFA_INFO.MIN_TX_TICK + 10) {
    VOFA_INFO.REAL_TX_FREQ = 1000.f / (HAL_GetTick() - tickstart);
    if (FireWater_factory(fbuf) == 0) {
      HAL_UART_Transmit(huart, (uint8_t *)vofa_buffer, vofa_tx_data_len,
                        0xffffffff);
    }
    VOFA_INFO.MIN_TX_TICK =
        1000.f * vofa_tx_data_len * (1 + 8 + 0 + 1) / huart->Init.BaudRate;
    VOFA_INFO.MAX_TX_FREQ = 1000.f / VOFA_INFO.MIN_TX_TICK;
    tickstart = HAL_GetTick();
  }
}
void update_vofa_with_usb(float *fbuf, uint8_t float_nums) {
  if (float_nums > MAX_FLOATS_NUM)
    return;
  if (FireWater_factory(fbuf) == 0) {
#if defined(STM32F407xx)
    CDC_Transmit_FS((uint8_t *)vofa_buffer, vofa_tx_data_len);
#elif defined(STM32H723xx)
    CDC_Transmit_HS((uint8_t *)vofa_buffer, vofa_tx_data_len);
#endif
  }
}
