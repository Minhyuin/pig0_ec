/**
 * @file drv_uart.c
 * @author Lizi
 * @brief 串口驱动层
 * @version 0.1
 *
 * @copyright SCNU-PIONEER (c) 2022-2023
 *
 */
#include "drv_uart.h"
#include "drv_conf.h"
#include <string.h>

#if USE_LOG_UART
#include "tools/log.h"
// 串口输出字符函数
static UART_HandleTypeDef *log_uart_handle = NULL;
static int uart_putc(int ch) {
  HAL_UART_Transmit(log_uart_handle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
void uart_log_init(UART_HandleTypeDef *huart) {
  log_uart_handle = huart;
  log_init(uart_putc);
  LOG("LOG Module Initialized");
}
#endif
#if defined(STM32H723xx)
// https://zhuanlan.zhihu.com/p/720966722
// tips: H7的串口设置不要把overrun和idle事件混用，否则会导致数据丢失
// 反正默认别勾上overrun就行了
HAL_StatusTypeDef uart_recv_dma_H7multibuffer_init(UART_HandleTypeDef *huart,
                                                   uint32_t *DstAddress,
                                                   uint32_t *SecondMemAddress,
                                                   uint32_t DataLength) {
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
  huart->RxEventType = HAL_UART_RXEVENT_IDLE;
  huart->RxXferSize = DataLength;
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
  return HAL_DMAEx_MultiBufferStart(
      huart->hdmarx, (uint32_t)&huart->Instance->RDR, (uint32_t)DstAddress,
      (uint32_t)SecondMemAddress, DataLength);
}
#endif
uint8_t a_send_queue[128], i_tx_write = 0, i_tx_read = 0;
extern uint16_t psc_cnt;
/**
 * @brief      enable uart receive uart it and initialize DMA
 * @param[in]  huart   pointer to the uart handle
 * @param[in]  rx_buffer   pointer to the receive buffer
 * @param[in]  len   buffer length
 * @return     set HAL_OK or HAL_BUSY
 */
uint32_t debug_ut = 0;
HAL_StatusTypeDef uart_recv_dma_init(UART_HandleTypeDef *huart,
                                     uint8_t *rx_buffer, uint16_t len) {
  HAL_UART_StateTypeDef state = huart->RxState;
  if (state == HAL_UART_STATE_READY) {
    __HAL_LOCK(huart);
    huart->pRxBuffPtr = (uint8_t *)rx_buffer;
    huart->RxXferSize = len;
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    debug_ut = (uint32_t)RX_DR_REG(huart);
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&RX_DR_REG(huart),
                  (uint32_t)rx_buffer, len);
    // HAL_DMA_Start(huart->hdmarx, RX_DR_REG(huart),
    // (uint32_t)pData, Size);
    /*
     * Enable the DMA transfer for the receiver request by setting the DMAR bit
     * in the UART CR3 register
     */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    //__HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);

    __HAL_UNLOCK(huart);

    return HAL_OK;
  } else {
    return HAL_BUSY;
  }
}

// HAL_StatusTypeDef uart_ui_dma_init(UART_HandleTypeDef* huart,uint8_t*
// tx_buffer,uint16_t len){ 	while(HAL_DMA_GetState(huart->hdmatx) !=
// HAL_DMA_STATE_READY);
//     //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
//
//     __HAL_DMA_DISABLE(huart->hdmatx);

//    HAL_UART_Transmit_DMA(huart, buffer, length);
//}

/**
 * @brief      send a couple of characters to uart
 * @param[in]  huart   pointer to the uart handle
 * @param[in]  p_data   pointer to the data buffer
 * @param[in]  size   data length to send
 * @return     the remaining size unsend, zero for success
 */
uint32_t uart_send(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size) {
  uint32_t start_tick = HAL_GetTick();
  while (size) {
    WRITE_REG(RX_DR_REG(huart), *p_data);
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != SET) {
      if (HAL_GetTick() - start_tick > UART_TIME_OUT)
        return size;
    }
    size--;
    p_data++;
  }
  return 0;
  //	return HAL_UART_Transmit(huart,p_data,size,UART_TIME_OUT);
}

HAL_StatusTypeDef flag;
/**
 * @brief      send a couple of characters to uart (non-blocking)
 * @param[in]  huart   pointer to the uart handle
 * @param[in]  p_data   pointer to the data buffer
 * @param[in]  size   data length to send
 * @return      HAL status
 */
HAL_StatusTypeDef uart_send_async(UART_HandleTypeDef *huart, uint8_t *p_data,
                                  uint16_t size) {
#if defined(STM32F407xx)
  return HAL_UART_Transmit(huart, p_data, size, 150);
#elif defined(STM32H723xx)
  return HAL_UART_Transmit_DMA(huart, p_data, size);
#endif
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // __HAL_DMA_CLEAR_FLAG(huart->hdmatx,
  // __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx));
  // HAL_UART_DMAStop(huart);
#if defined(STM32H723xx)
  extern UART_HandleTypeDef DEBUG_UART_HANDLE;
  huart->gState = HAL_UART_STATE_READY;
  if (huart == &DEBUG_UART_HANDLE) {
    __HAL_DMA_CLEAR_FLAG(DEBUG_UART_HANDLE.hdmatx,
                         DMA_FLAG_TCIF0_4); // 0或4通道
    HAL_UART_DMAStop(
        &DEBUG_UART_HANDLE); // 传输完成以后关闭串口DMA,缺了这一句会死机
  }
#endif
}
#if defined(STM32H723xx)
#include "../Base/rc.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
#if 0
  if (huart->Instance == UART5) {
    if (Size <= BUFF_SIZE) {
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff,
                                   BUFF_SIZE * 2); // 接收完毕后重启
      sbus_frame_parse(&remoter, rx_buff);
      //			memset(rx_buff, 0, BUFF_SIZE);
    } else // 接收数据长度大于BUFF_SIZE，错误处理
    {
      HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buff,
                                   BUFF_SIZE * 2); // 接收完毕后重启
      memset(rx_buff, 0, BUFF_SIZE);
    }
  }
#endif
  if (huart->Instance == RC_UART) {
    rc_uart_idle_handle(huart);
  }
#if USE_EXT_IMU_UART
  if (huart->Instance == EXT_IMU_UART) {
    ext_uart_imu_idle_handle(huart);
  }
#endif
}
#endif
