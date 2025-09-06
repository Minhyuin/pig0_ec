/**
 * @file super_cap.c
 *
 * @brief 超级电容管理
 *
 *
 * @copyright SCNU-PIONEER (c) 2023-2024
 *
 */
#include "Base/super_cap.h"
#include "APP/bus_detect.h"
#include "Algorithm/util.h"
#include "string.h"

// setup super cap default value
cap_data_t cap_data = {.input_voltage = 24.0f,
                       .cap_voltage = 21.9f,
                       .power_set = 50,
                       .SUPERCAP_ENABLE_FLAG = 0};

static uint16_t cap_rx_lost = SUPER_CAP_RX_MAX_LOST;

/**
 * @brief parse can data from super cap board and store it
 * @param[in]  rx_header  the CAN header
 * @param[in]  rx_buffer  raw data(8 bytes) received
 * @return HAL_OK if the id is match otherwise HAL_ERROR
 */
float cap_data_rfreq = 0;
raw_cap_data_t raw_cap_data;
float VCAP_MAX = 0;
HAL_StatusTypeDef parse_cap_data(CAN_RxHeaderTypeDef *rx_header,
                                 uint8_t *rx_buffer) {
  memcpy(&raw_cap_data, rx_buffer, CAN_DATA_LEN);
  if (rx_header->StdId == CAP_RESPONSE_ID) {
    get_data_refresh_freq(cap_data.input_voltage, cap_data_rfreq);

    cap_data.input_voltage =
        24.f + (int16_t)raw_cap_data.bus_voltage_s24m100 / 100.0f;
    cap_data.cap_voltage = (uint16_t)raw_cap_data.cap_voltage_m70 / 70.0f;
    VCAP_MAX -= 0.001f;
    if (cap_data.cap_voltage > VCAP_MAX) {
      VCAP_MAX = cap_data.cap_voltage;
    }
    LIMIT_MIN_MAX(VCAP_MAX, 24.5f, 26.f);
    cap_data.input_current =
        (uint16_t)raw_cap_data.input_current_m1000 / 1000.0f;
    cap_data.output_current =
        cap_data.input_current -
        (int16_t)raw_cap_data.charge_current_m1000 / 1000.0f;
    cap_data.power_set = (uint8_t)raw_cap_data.set_power_in;

    cap_data.SUPERCAP_ENABLE_FLAG = raw_cap_data.SUPERCAP_ENABLE_FLAG;
    cap_data.LOW_VOLTAGE_FLAG = raw_cap_data.LOW_VOLTAGE_FLAG;

    cap_rx_lost = 0;
    return HAL_OK;
  } else
    return HAL_ERROR;
}

/**
 * @brief  get a copy of super cap response data
 * @return a copy of data in cap_data_t
 */
cap_data_t *get_cap_data(void) {
  //__HAL_CAN_DISABLE_IT(&SUPER_CAP_CAN_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING);
  //__HAL_CAN_ENABLE_IT(&SUPER_CAP_CAN_HANDLE, CAN_IT_RX_FIFO0_MSG_PENDING);
  return &cap_data;
}

/**
 * @brief set the super cap power limit
 * @param[in]  hcan  the CAN handler to transmit raw data
 * @param[in]  set_power  the expected power limit
 * @return HAL_OK if transmission was success otherwise HAL_ERROR
 */
HAL_StatusTypeDef ret = HAL_OK;
uint32_t super_can_sent_freq = 1000;

HAL_StatusTypeDef set_cap_power(CAN_HandleTypeDef *hcan, float set_power,
                                uint8_t ENABLE_SUPERCAP_FLAG) {
  static uint32_t cap_tickstart = 0;
  if (HAL_GetTick() - cap_tickstart >= 1000.f / super_can_sent_freq) {

    cap_cmd_t cap_cmd = {0};
    cap_cmd.power_set = set_power;
    cap_cmd.ENABLE_SUPERCAP = ENABLE_SUPERCAP_FLAG;
    uint8_t tx_buffer[CAN_DATA_LEN];
    memcpy(tx_buffer, &cap_cmd, CAN_DATA_LEN);
    ret = can_transmit_data(hcan, CAP_TRANSMIT_ID, tx_buffer, CAN_DATA_LEN, 0);
    cap_tickstart = HAL_GetTick();
  }
  return ret;
}

void inc_cap_rx_lost(void) {
  if (cap_rx_lost < SUPER_CAP_RX_MAX_LOST)
    cap_rx_lost++;
  else
    cap_data.SUPERCAP_ENABLE_FLAG = 0;
}
