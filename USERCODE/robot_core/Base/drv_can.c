/**
 * @file drv_can.c
 *
 *
 * @brief can应用层
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "drv_can.h"

#if MOTOR_CAN_ENABLE == 1

#include "APP/boards_interact.h"
#include "Base/ext_imu.h"
#include "Base/super_cap.h"
#include "Devices/MOTOR/motor_headers.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#if defined(STM32H723xx)
// https://blog.csdn.net/wallace89/article/details/110578827
// cube里
// fdcan1.Message Ram Offset=0
// fdcan2.Message Ram Offset= hfdcan1.msgRam.EndAddress - SRAMCAN_BASE
// fdcan3.Message Ram Offset= 2*(hfdcan1.msgRam.EndAddress - SRAMCAN_BASE)
// 若启用三个fdcan 则三者平均占用内存

HAL_StatusTypeDef FDCAN_INIT_FLAG = HAL_OK;
// 用fdcan_user_init的话这玩意在第二次调用后变成HAL_ERROR
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
#define Write_ID_Into_Filter(index, id1, id2)                                  \
  do {                                                                         \
    fdcan_filter.FilterIndex = (index);                                        \
    fdcan_filter.FilterID1 = (id1);                                            \
    fdcan_filter.FilterID2 = (id2);                                            \
    FDCAN_INIT_FLAG |= HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);          \
  } while (0)
uint8_t FILTER_Index[2] = {0};
/**
 * @brief  	  init can filter, start can, enable can rx interrupt
 * @param[in] hcan  pointer to a FDCAN_HandleTypeDef structure that contains
 *                  the configuration information for the specified CAN.
 * @return    HAL_OK if success otherwise HAL_ERROR
 */
HAL_StatusTypeDef fdcan_user_init(FDCAN_HandleTypeDef *hfdcan,
                                  uint32_t FliterId[4], bool FIFO) {
  /******edit filter*******/
  FDCAN_FilterTypeDef fdcan_filter;
  // FDCAN_INIT_FLAG |= HAL_FDCAN_Stop(hfdcan);
  /**
   * C610:    0x201~0x208
   * C620:    0x201~0x208
   * GM6020:  0x205~0x20A
   * 反馈帧:  0x211~0x20A,共10个
   * ctrl_id: 0x200,0x1ff,0x2ff
   */
  fdcan_filter.IdType = FDCAN_STANDARD_ID;
  fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
  // fdcan_filter.FilterType = FDCAN_FILTER_MASK;
  for (uint8_t i = 0; i < 2; i++) {
    /******write id*******/
    if (FIFO == 0) {
      /******FIFO0 init*******/
      fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
      Write_ID_Into_Filter(FILTER_Index[0]++, FliterId[2 * i],
                           FliterId[1 + 2 * i]);
      FDCAN_INIT_FLAG |= HAL_FDCAN_ActivateNotification(
          hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
      // 使能中断,第三个参数n与使能发送中断相关,我们无需做处理
    } else if (FIFO == 1) {
      /******FIFO1 init*******/
      fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
      Write_ID_Into_Filter(FILTER_Index[1]++, FliterId[2 * i],
                           FliterId[1 + 2 * i]);
      FDCAN_INIT_FLAG |= HAL_FDCAN_ActivateNotification(
          hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
    /******start up!*******/
    FDCAN_INIT_FLAG |=
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
    FDCAN_INIT_FLAG |=
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    FDCAN_INIT_FLAG |= HAL_FDCAN_Start(hfdcan);
  }
  return FDCAN_INIT_FLAG;
}

HAL_StatusTypeDef fdcan_user_init_all(FDCAN_HandleTypeDef *hfdcan) {
  motors_t *motor = get_motors_ptr();
  /******edit filter*******/
  FDCAN_FilterTypeDef fdcan_filter;
  fdcan_filter.IdType = FDCAN_STANDARD_ID;
  fdcan_filter.FilterType = FDCAN_FILTER_DUAL;
  /******write id*******/
  if (hfdcan == &hfdcan1) {
    /******HIP_MOTORS init*******/
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    Write_ID_Into_Filter(FILTER_Index[0]++, 0x000, 0x000);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    // 使能中断,第三个参数n与使能发送中断相关,我们无需做处理
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  }

  if (hfdcan == &hfdcan2) {
    /******WHEEL_MOTORS init*******/
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    Write_ID_Into_Filter(FILTER_Index[1]++, 0x000, 0x000);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  }
  /******start up!*******/
  FDCAN_INIT_FLAG |=
      HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
  FDCAN_INIT_FLAG |= HAL_FDCAN_Start(hfdcan);
  return FDCAN_INIT_FLAG;
}

uint32_t fdcan_error_cnt[3] = {0};
// 协议状态错误参考:https://blog.csdn.net/jcjx0315/article/details/70137271
FDCAN_ProtocolStatusTypeDef FDCAN_PS;
#define FDCAN_ERROR_CNT_MAX (M_TIM_FREQ)
/**
 * @brief FDCAN发送函数
 *
 * @param hfdcan
 * @param stdid
 * @param tx_data
 * @return HAL_StatusTypeDef
 * @note  在FDCAN专属的消息RAM中，存在32个TX缓冲区
 */
HAL_StatusTypeDef fdcan_transmit_data(FDCAN_HandleTypeDef *hfdcan,
                                      uint16_t stdid, uint8_t *tx_data,
                                      uint8_t size, bool is_fdcan) {
  FDCAN_TxHeaderTypeDef tx_header = {0};
  tx_header.Identifier = stdid;             //
  tx_header.IdType = FDCAN_STANDARD_ID;     //
  tx_header.TxFrameType = FDCAN_DATA_FRAME; //
  switch (size) {
  case 1:
    tx_header.DataLength = FDCAN_DLC_BYTES_1;
    break;
  case 2:
    tx_header.DataLength = FDCAN_DLC_BYTES_2;
    break;
  case 3:
    tx_header.DataLength = FDCAN_DLC_BYTES_3;
    break;
  case 4:
    tx_header.DataLength = FDCAN_DLC_BYTES_4;
    break;
  case 5:
    tx_header.DataLength = FDCAN_DLC_BYTES_5;
    break;
  case 6:
    tx_header.DataLength = FDCAN_DLC_BYTES_6;
    break;
  case 7:
    tx_header.DataLength = FDCAN_DLC_BYTES_7;
    break;
  case 8:
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    break;
  case 12:
    tx_header.DataLength = FDCAN_DLC_BYTES_12;
    break;
  case 16:
    tx_header.DataLength = FDCAN_DLC_BYTES_16;
    break;
  case 20:
    tx_header.DataLength = FDCAN_DLC_BYTES_20;
    break;
  case 24:
    tx_header.DataLength = FDCAN_DLC_BYTES_24;
    break;
  case 32:
    tx_header.DataLength = FDCAN_DLC_BYTES_32;
    break;
  case 48:
    tx_header.DataLength = FDCAN_DLC_BYTES_48;
    break;
  case 64:
    tx_header.DataLength = FDCAN_DLC_BYTES_64;
    break;
  default:
    break;
  }
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // FDCAN_ESI_PASSIVE;
  if (is_fdcan) {
    tx_header.BitRateSwitch = FDCAN_BRS_ON; //
    tx_header.FDFormat = FDCAN_FD_CAN;      //
  } else {
    tx_header.BitRateSwitch = FDCAN_BRS_OFF; //
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;  //
  }
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;
  uint8_t can_channel = (hfdcan == &hfdcan1)   ? 0
                        : (hfdcan == &hfdcan2) ? 1
                        : (hfdcan == &hfdcan3) ? 2
                                               : 0;
  // HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data) != HAL_OK) {
#if 1
    if (fdcan_error_cnt[can_channel] >= FDCAN_ERROR_CNT_MAX) {
      // HAL_FDCAN_GetProtocolStatus(hfdcan, &FDCAN_PS);
      fdcan_error_cnt[can_channel] = 0;
      HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
      HAL_FDCAN_DeactivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE);
      HAL_FDCAN_Stop(hfdcan);
      uint16_t i = 0xFF;
      while (i--)
        ;
      HAL_FDCAN_Start(hfdcan);
      HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
      HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
#endif
    fdcan_error_cnt[can_channel]++;
    return HAL_ERROR;
  } else {
    fdcan_error_cnt[can_channel] = 0;
  }
  return HAL_OK;
}

/**
 * @brief 	can FIFO0的接收中断回调函数
 * */
uint16_t Label_FIFO0;
uint8_t CAN_RX_FIFO0_BUF[8] = {0}; // FIFO0收到的数据
FDCAN_HandleTypeDef *p_fdcan_fifo0 = NULL;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, CAN_RX_FIFO0_BUF);
  p_fdcan_fifo0 = hfdcan;
  Label_FIFO0 = rx_header.Identifier;

  // 处理电机数据
  parse_motor_data(hfdcan, &rx_header, CAN_RX_FIFO0_BUF);
}

/**
 * @brief 	can FIFO1的接收中断回调函数
 * */
uint16_t Label_FIFO1;
uint8_t CAN_RX_FIFO1_BUF[8] = {0}; // FIFO0收到的数据
FDCAN_HandleTypeDef *p_fdcan_fifo1 = NULL;
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  FDCAN_RxHeaderTypeDef rx_header;
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, CAN_RX_FIFO1_BUF);
  p_fdcan_fifo1 = hfdcan;
  Label_FIFO1 = rx_header.Identifier;

  // 处理电机数据
  parse_motor_data(hfdcan, &rx_header, CAN_RX_FIFO1_BUF);
#if USE_EXT_CAN_IMU
  parse_ext_imu_can_data(rx_header.StdId, CAN_RX_FIFO1_BUF);
#endif
}
#elif defined(STM32F407xx)
/**
 * @brief  can fifo 0 rx callback, get motor feedback info
 * @param[in]  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @note this is a weak function in HAL
 * @return None
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_buffer[CAN_DATA_LEN];
  // 判断读取FIFO0是否有效
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer) ==
      HAL_OK) {
#if USE_EXT_CAN_IMU
    // 处理外置IMU响应数据
    parse_motor_data(hcan, &rx_header, rx_buffer);
#else
    // 处理电机数据
    parse_motor_data(hcan, &rx_header, rx_buffer);
#endif
#if BOARDS_MODE
    board_interact_parse(rx_buffer, rx_header.StdId);
#endif
  }
}

/**
 * @brief  can fifo 1 rx callback, get motor feedback info
 * @param[in]  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @return None
 * @note this is a weak function in HAL
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_buffer[CAN_DATA_LEN];
  // 判断读取FIFO1是否有效
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_buffer) ==
      HAL_OK) {
#if USE_SUPER_CAP
    // 处理超级电容响应数据
    if (rx_header.StdId == CAP_RESPONSE_ID) {
      parse_cap_data(&rx_header, rx_buffer);
    } else {
      parse_motor_data(hcan, &rx_header, rx_buffer);
    }
#else
    parse_motor_data(hcan->Instance, &rx_header, rx_buffer);
#endif
#if USE_EXT_CAN_IMU
    parse_ext_imu_can_data(rx_header.StdId, rx_buffer);
#endif
#if BOARDS_MODE
    board_interact_parse(rx_buffer, rx_header.StdId);
#endif
  }
}

/**
 * @brief  init can filter, start can, enable can rx interrupt
 * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @return HAL_OK if success otherwise HAL_ERROR
 */
HAL_StatusTypeDef CAN_INIT_FLAG = HAL_OK;
HAL_StatusTypeDef can_user_init(CAN_HandleTypeDef *hcan, uint32_t FilterId[4],
                                bool FIFO) {
  static uint8_t can1_init_FilterNum = 0;
  static uint8_t can2_init_FilterNum = 0;
  if ((hcan != &hcan1 && hcan != &hcan2) ||
      (hcan == &hcan1 && can1_init_FilterNum > 13) ||
      (hcan == &hcan2 && can2_init_FilterNum > 13))
    return HAL_ERROR;

  CAN_FilterTypeDef can_filter;
  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterActivation = ENABLE; // enable can filter
  can_filter.SlaveStartFilterBank = 14;
  // can的FilterBank参数是用来配置过滤器组的(标准库叫做FilterNumber)
  // 双CAN系列的产品有27个滤波器,其中can1的滤波器号为0~13,can2为14~27
  // 为了一路can配置8个id(list模式)(FIFO0与FIFO1各自4个)
  // 就不应该反复塞到0 or 14滤波器里(应该吧?)
  // 这里就采用了起始滤波器+FilterNum选定通道
  if (hcan->Instance == CAN1) {
    can_filter.FilterBank = 0 + can1_init_FilterNum;
    can1_init_FilterNum++;
  } else {
    can_filter.FilterBank = 14 + can2_init_FilterNum;
    can2_init_FilterNum++;
  }
  uint32_t real_filterId[4];
  for (uint8_t i = 0; i < 4; i++) {
    if (FilterId[i] != 0)
      real_filterId[i] = FilterId[i] << 5;
    else
      real_filterId[i] = 0;
  }
  //	for(uint8_t i = 0;i < 4; i++)
  //		real_filterId[i] = FilterId[i]<<5;
  can_filter.FilterIdHigh = real_filterId[0];
  can_filter.FilterIdLow = real_filterId[1];
  can_filter.FilterMaskIdHigh = real_filterId[2];
  can_filter.FilterMaskIdLow = real_filterId[3];
  can_filter.FilterFIFOAssignment = FIFO;
  CAN_INIT_FLAG |= HAL_CAN_ConfigFilter(hcan, &can_filter);

  if (FIFO == CAN_FILTER_FIFO0)
    //    CAN_INIT_FLAG |=
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  else
    //    CAN_INIT_FLAG |=
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_BUSOFF);

  CAN_INIT_FLAG |= HAL_CAN_Start(hcan); // start can
  return CAN_INIT_FLAG;
}

HAL_StatusTypeDef can_transmit_data(CAN_HandleTypeDef *hcan, uint16_t stdid,
                                    uint8_t *tx_data, uint8_t size,
                                    bool is_fdcan) {
  UNUSED(is_fdcan);
  CAN_TxHeaderTypeDef tx_header;
  uint32_t can_mailbox;
  tx_header.DLC = size;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.StdId = stdid;
  tx_header.TransmitGlobalTime = DISABLE;
  return HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &can_mailbox);
}

#define MOTOR_ID_OFFSET 0x201U
HAL_StatusTypeDef old_can_user_init(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef can_filter;

  can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 0) << 5;
  can_filter.FilterIdLow = (MOTOR_ID_OFFSET + 1) << 5;
  can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 2) << 5;
  can_filter.FilterMaskIdLow = (MOTOR_ID_OFFSET + 3) << 5;
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterActivation = ENABLE;
  can_filter.SlaveStartFilterBank = 14;

  if (hcan->Instance == CAN2) {
    can_filter.FilterBank = 14;
  } else {
    can_filter.FilterBank = 0;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) == HAL_OK) { // init can filter
    // no error HAL_CAN_ERROR_NOT_INITIALIZED
    can_filter.FilterBank = can_filter.FilterBank + 1;
    can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 4) << 5; // 32位ID，高16位
    can_filter.FilterIdLow = (MOTOR_ID_OFFSET + 5) << 5;  // 低16位
    can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 6) << 5;
    can_filter.FilterMaskIdLow = (MOTOR_ID_OFFSET + 7) << 5;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    HAL_CAN_ConfigFilter(hcan, &can_filter);

    can_filter.FilterBank = can_filter.FilterBank + 1;
    can_filter.FilterIdHigh = (MOTOR_ID_OFFSET + 8) << 5; // 32位ID，高16位
    can_filter.FilterIdLow = (MOTOR_ID_OFFSET + 9) << 5;  // 低16位
    can_filter.FilterMaskIdHigh = (MOTOR_ID_OFFSET + 10) << 5;
    can_filter.FilterMaskIdLow = 0;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    HAL_CAN_ConfigFilter(hcan, &can_filter);

    HAL_CAN_ActivateNotification(
        hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // enable can rx interrupt
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_BUSOFF);

    return HAL_CAN_Start(hcan); // start can
  } else {
    return HAL_ERROR;
  }
}
#endif

#endif
