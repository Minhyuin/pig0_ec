#ifndef _BOARDS_INTERACT_H
#define _BOARDS_INTERACT_H

#include "APP/robot.h"
#include "Base/rc.h"

/**
 * @brief 用于半精度浮点数相互转换
 */
typedef struct {
  unsigned int mantissa : 10; // 尾数
  unsigned int exponent : 5;  // 指数
  unsigned int sign : 1;      // 符号位
} float16_t;
void transfer_1632_test(float fin, float *fout);

#define BOARD_INTERACT_FREQ 10
#define G2C_NUM 2
#define C2G_NUM 1

#pragma pack(push, 1)
/**
 * 以下是根据框架的控制逻辑决定的
 * 由于基于CAN帧，所以数据包大小最大为64bit
 * 先弄清需要发送给机构什么信息 和 机构需要反馈什么给主控
 */
typedef struct {
  int16_t rocker_rx : 16;
  int16_t rocker_ry : 16;
  int16_t chassis_follow_err_ang : 16;
  uint8_t c_tag : 4;
  uint8_t s_tag : 4;
  uint8_t robot_control_flag : 1;
  uint8_t fire_flag : 1;
  uint8_t rush_flag : 1;
  uint8_t use_supercap_flag : 1;
  uint8_t gimbal_launch_flag : 1;
  uint32_t reserved : 3;
} G2C_PACK1;
typedef struct {
  int16_t yaw_delta : 16;
  int16_t reserved1 : 16;
  uint32_t reserved2 : 32;
} G2C_PACK2;
typedef union {
  G2C_PACK1 pack1;
  G2C_PACK2 pack2;
} G2C_BUF;

typedef struct {
  uint16_t HP : 16;
  uint16_t game_time : 16;
  uint8_t robot_id : 8;
  uint16_t cap_energy : 16;
  uint8_t charge_request_flag : 1;
  uint8_t chassis_launch_flag : 1;
  uint32_t reserved : 7;
} C2G_PACK1;
// typedef struct{
//     uint32_t reserved1:32;
//     uint32_t reserved2:32;
// } C2G_PACK2;

// typedef struct{
//     uint32_t bullet_speed:32;
//     uint8_t bullet_freq:8;
//     uint32_t reserved:24;
// } C2G_PACK3;
typedef union {
  C2G_PACK1 pack1;
} C2G_BUF;
#pragma pack(pop)
// 云台发送给底盘的包
typedef struct G2C_t {
  G2C_BUF buffer;
  void (*assemble_func)(struct G2C_t *);
  void (*parse_func)(uint8_t *buffer);
  CAN_HandleTypeDef *pcan;
  uint16_t stdid;
  uint32_t refresh_cnt;
  LPF_t refresh_filter;
  float refresh_freq;
} G2C_t;

// 底盘发送给云台的包
typedef struct C2G_t {
  C2G_BUF buffer;
  void (*assemble_func)(struct C2G_t *);
  void (*parse_func)(uint8_t *buffer);
  CAN_HandleTypeDef *pcan;
  uint16_t stdid;
  uint32_t refresh_cnt;
  LPF_t refresh_filter;
  float refresh_freq;
} C2G_t;

void board_interact_init(void);
void board_interact_send(void);
void board_interact_parse(uint8_t *buffer, uint32_t stdid);
void update_boards_interact_status(void);

#endif
