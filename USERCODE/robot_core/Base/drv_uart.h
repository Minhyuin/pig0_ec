#ifndef _UART_DRV_H
#define _UART_DRV_H

#include "drv_conf.h"
#include HAL_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif
#define UART_TIME_OUT 5
void uart_log_init(UART_HandleTypeDef *huart);
#if defined(STM32H723xx)
#define RX_DR_REG(huart) ((huart)->Instance->RDR)
HAL_StatusTypeDef uart_recv_dma_H7multibuffer_init(UART_HandleTypeDef *huart,
                                                   uint32_t *DstAddress,
                                                   uint32_t *SecondMemAddress,
                                                   uint32_t DataLength);
#elif defined(STM32F407xx)
#define RX_DR_REG(huart) ((huart)->Instance->DR)
#endif
uint32_t uart_send(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size);
HAL_StatusTypeDef uart_send_async(UART_HandleTypeDef *huart, uint8_t *p_data,
                                  uint16_t size);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

HAL_StatusTypeDef uart_recv_dma_init(UART_HandleTypeDef *huart,
                                     uint8_t *rx_buffer, uint16_t len);
// HAL_StatusTypeDef uart_ui_dma_init(UART_HandleTypeDef* huart,uint8_t*
// tx_buffer,uint16_t len);
#ifdef __cplusplus
}
#endif
#endif
