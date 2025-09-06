#ifndef _DRV_CAN_H
#define _DRV_CAN_H

#include "drv_conf.h"
#include HAL_INCLUDE
#ifdef __cplusplus
extern "C" {
#endif
#if MOTOR_CAN_ENABLE == 1
#if defined(STM32H723xx)
HAL_StatusTypeDef fdcan_user_init(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t FliterId[4], bool FIFO);
HAL_StatusTypeDef fdcan_transmit_data(FDCAN_HandleTypeDef *hfdcan,
                                      uint16_t stdid, uint8_t *tx_data,
                                      uint8_t size, bool is_fdcan);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs);
#ifndef CAN_HandleTypeDef
#define CAN_HandleTypeDef FDCAN_HandleTypeDef
#endif
#ifndef CAN_RxHeaderTypeDef
#define CAN_RxHeaderTypeDef FDCAN_RxHeaderTypeDef
#endif
#ifndef CAN_TxHeaderTypeDef
#define CAN_TxHeaderTypeDef FDCAN_TxHeaderTypeDef
#endif

#ifndef hcan1
#define hcan1 hfdcan1
#endif
#ifndef hcan2
#define hcan2 hfdcan2
#endif
#ifndef hcan3
#define hcan3 hfdcan3
#endif

#ifndef can_transmit_data
#define can_transmit_data fdcan_transmit_data
#endif

#ifndef can_user_init
#define can_user_init fdcan_user_init
#endif

#ifndef StdId
#define StdId Identifier
#endif
#elif defined(STM32F407xx)
HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef *hcan, uint32_t FilterId[4],
                                bool FIFO);
HAL_StatusTypeDef can_transmit_data(CAN_HandleTypeDef *hcan, uint16_t stdid,
                                    uint8_t *tx_data, uint8_t size,
                                    bool is_fdcan);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

HAL_StatusTypeDef old_can_user_init(CAN_HandleTypeDef *hcan);
#endif
#endif
#ifdef __cplusplus
}
#endif
#endif
