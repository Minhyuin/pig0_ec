#ifndef _NAV_H_
#define _NAV_H_

#include "../drv_conf.h"

#include HAL_INCLUDE

#pragma pack(push, 1)
typedef struct {
  uint8_t header;
  uint8_t reserved;
  uint16_t checksum;
} nav_req_t;
typedef struct {
  uint8_t header;
  uint8_t reserved;
  float nav_vx;
  float nav_vy;
  float nav_wz;
  uint16_t checksum;
} nav_ctrl_t;
#pragma pack(pop)

uint8_t parse_nav_data(uint8_t *buf, uint16_t len);
void send_nav_request(void *data);

nav_ctrl_t *get_nav_ctrl_data_ptr(void);

#endif // !_NAV_H_
