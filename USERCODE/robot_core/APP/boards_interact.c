/**
 * @file boards_interct.c
 *
 * @brief 板间通信
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "boards_interact.h"
#include "Base/bsp_dwt.h"
#define uint16_t unsigned short
#if BOARDS_MODE
/*计时变量*/
uint32_t interact_dwt_cnt = 0;
float interact_dt = 1.f / INTERACT_TIM_FREQ;

static int16_t float32_to_int16(float f) {
  float16_t h;
  uint16_t out = 0;
  if (f == 0)
    out = 0;
  else {
    // 提取符号、指数和尾数
    unsigned int sign = (f < 0) ? 1 : 0;
    unsigned int exponent =
        ((*(unsigned int *)&f >> 23) & 0xFF) - 127 + 15;            // 调整指数
    unsigned int mantissa = (*(unsigned int *)&f & 0x7FFFFF) >> 13; // 截取尾数
    // 处理溢出
    if (exponent >= 0x1F) {
      exponent = 0x1F;
      mantissa = 0; // 处理无穷大情况
    } else if (exponent <= 0) {
      exponent = 0;
      mantissa = 0; // 处理零和非规格化数
    }
    h.sign = sign;
    h.exponent = exponent;
    h.mantissa = mantissa;
    memcpy(&out, &h, 2);
  }
  return out;
}
static float int16_t_to_float32(int16_t u) {
  float16_t h;
  if (u == 0)
    return 0.f;
  else {
    memcpy(&h, &u, 2);
    unsigned int sign = h.sign;
    unsigned int exponent = h.exponent - 15 + 127; // 还原指数
    unsigned int mantissa = h.mantissa << 13;      // 补齐尾数
    if (exponent >= 0xFF) {
      exponent = 0xFF; // 处理无穷大
      mantissa = 0;    // 设置尾数为0
    }
    unsigned int f = (sign << 31) | (exponent << 23) | mantissa;
    return *(float *)&f;
  }
}
// only for test
void transfer_1632_test(float fin, float *fout) {
  int16_t temp = 0;
  temp = float32_to_int16(fin);
  *fout = int16_t_to_float32(temp);
}

extern CAN_HandleTypeDef BOARDS_INTERACT_CAN_HANDLE;

extern ext_game_robot_status_t game_robot_status;
extern ext_game_status_t game_status;
extern ext_shoot_data_t shoot_data;
extern ext_power_heat_data_t power_heat_data;
extern chassis_power_lim_t chassis_power_lim;

extern float rocker_rx;
extern float rocker_ry;
extern int16_t chassis_follow_err_ang;
extern int16_t yaw_delta;

G2C_t g2c_ins[G2C_NUM]; // 实例对象
C2G_t c2g_ins[C2G_NUM];

/**
 * @brief 装载函数
 */
static void g2c_pack1_load(G2C_t *g2c) {
  G2C_PACK1 *pack = &g2c->buffer.pack1;
  /* assemble space */
  robot_t *robot = get_robot_ptr();
  rc_ctrl_t *dt7_data = get_dt7_data_ptr();
  pack->rocker_rx = float32_to_int16(rocker_rx);
  pack->rocker_ry = float32_to_int16(rocker_ry);
  pack->robot_control_flag = robot->robot_control_flag;
  pack->c_tag = robot->chassis_tag;
  pack->s_tag = robot->shooter_tag;
  pack->gimbal_launch_flag = *get_gimbal_reset_flag_ptr();
  pack->fire_flag = robot->fire_flag;
  pack->rush_flag = robot->rush_flag;
  pack->use_supercap_flag = robot->use_supercap_flag;
  pack->chassis_follow_err_ang = chassis_follow_err_ang;
  /* assemble space */
  memcpy(&g2c->buffer, pack, 8);
}
static void g2c_pack2_load(G2C_t *g2c) {
  G2C_PACK2 *pack = &g2c->buffer.pack2;
  /* assemble space */
  pack->yaw_delta = yaw_delta;
  /* assemble space */
  memcpy(&g2c->buffer, pack, 8);
}
static void c2g_pack1_load(C2G_t *c2g) {
  C2G_PACK1 *pack = &c2g->buffer.pack1;
  /* assemble space */
  robot_t *robot = get_robot_ptr();
  pack->chassis_launch_flag = *get_chassis_reset_flag_ptr();
  pack->HP = game_robot_status.current_HP;
  pack->robot_id = game_robot_status.robot_id;
  pack->game_time = game_status.stage_remain_time;
  pack->cap_energy = float32_to_int16(chassis_power_lim.expt_cap_energy_ratio);
  pack->charge_request_flag = robot->charge_request_flag;
  /* assemble space */
  memcpy(&c2g->buffer, pack, 8);
}
// static void c2g_pack2_load(C2G_t *c2g) {
//     C2G_PACK2 *pack = &c2g->buffer.pack2;
// /* assemble space */
// /* assemble space */
//     memcpy(&c2g->buffer, pack, 8);
// }

// static void c2g_pack3_load(C2G_t *c2g) {
//     C2G_PACK3 *pack = &c2g->buffer.pack3;
// /* assemble space */
//     pack.bullet_freq = shoot_data.bullet_freq;
//     pack.bullet_speed = shoot_data.bullet_speed;
// /* assemble space */
//     memcpy(&c2g->buffer, pack, 8);
// }

/**
 * @brief 解析函数
 */
static void g2c_pack1_parse(uint8_t *buffer) {
  memcpy(&g2c_ins[0].buffer, buffer, 8);
  G2C_PACK1 *pack = (G2C_PACK1 *)&g2c_ins[0].buffer;
  /* parse space */
  robot_t *robot = get_robot_ptr();
  rc_ctrl_t *dt7_data = get_dt7_data_ptr();
  rocker_rx = int16_t_to_float32(pack->rocker_rx);
  rocker_ry = int16_t_to_float32(pack->rocker_ry);
  robot->robot_control_flag = pack->robot_control_flag;
  robot->chassis_tag = (chassis_tag_t)pack->c_tag;
  robot->shooter_tag = (shooter_tag_t)pack->s_tag;
  *get_gimbal_reset_flag_ptr() = pack->gimbal_launch_flag;
  robot->fire_flag = pack->fire_flag;
  robot->rush_flag = pack->rush_flag;
  robot->use_supercap_flag = pack->use_supercap_flag;
  chassis_follow_err_ang = pack->chassis_follow_err_ang;
  /* parse space */
}
static void g2c_pack2_parse(uint8_t *buffer) {
  memcpy(&g2c_ins[1].buffer, buffer, 8);
  G2C_PACK2 *pack = (G2C_PACK2 *)&g2c_ins[1].buffer;
  /* parse space */
  yaw_delta = pack->yaw_delta;
  /* parse space */
}

static void c2g_pack1_parse(uint8_t *buffer) {
  memcpy(&c2g_ins[0].buffer, buffer, 8);
  C2G_PACK1 *pack = (C2G_PACK1 *)&c2g_ins[0].buffer;
  /* parse space */
  robot_t *robot = get_robot_ptr();
  *get_chassis_reset_flag_ptr() = pack->chassis_launch_flag;
  game_robot_status.current_HP = pack->HP;
  game_robot_status.robot_id = pack->robot_id;
  game_status.stage_remain_time = pack->game_time;
  chassis_power_lim.expt_cap_energy_ratio =
      int16_t_to_float32(pack->cap_energy);
  robot->charge_request_flag = pack->charge_request_flag;
  /* parse space */
}
// static void c2g_pack3_parse(uint8_t *buffer) {
//    memcpy(&c2g_ins[2].buffer, buffer, 8);
//     C2G_PACK3 *pack = (C2G_PACK3 *)&c2g_ins[2].buffer;
// /* parse space */
//     shoot_data.bullet_freq = pack->bullet_freq;
//     shoot_data.bullet_speed = pack->bullet_speed;
// /* parse space */
// }

/**
 * @brief 板间通信初始化
 */
void board_interact_init(void) {
  // 初始化通讯协议 相关属性预装载
  LPF_t refresh_filter = (LPF_t){.fc = 1.f, .ts = 1.f / DETECT_TIM_FREQ};

  g2c_ins[0] = (G2C_t){.assemble_func = &g2c_pack1_load,
                       .parse_func = &g2c_pack1_parse,
                       .pcan = &BOARDS_INTERACT_CAN_HANDLE,
                       .stdid = G2C_CAN_BASE_ID + 1,
                       .refresh_filter = refresh_filter};
  g2c_ins[1] = (G2C_t){.assemble_func = &g2c_pack2_load,
                       .parse_func = &g2c_pack2_parse,
                       .pcan = &BOARDS_INTERACT_CAN_HANDLE,
                       .stdid = G2C_CAN_BASE_ID + 2,
                       .refresh_filter = refresh_filter};
  can_user_init(&BOARDS_INTERACT_CAN_HANDLE,
                (uint32_t[]){g2c_ins[0].stdid, g2c_ins[1].stdid, 0, 0}, 1);

  c2g_ins[0] = (C2G_t){.assemble_func = &c2g_pack1_load,
                       .parse_func = &c2g_pack1_parse,
                       .pcan = &BOARDS_INTERACT_CAN_HANDLE,
                       .stdid = C2G_CAN_BASE_ID + 1,
                       .refresh_filter = refresh_filter};
  // c2g_ins[1] = (C2G_t){.assemble_func = &c2g_pack2_load, .pcan =
  // &BOARDS_INTERACT_CAN_HANDLE, .stdid = C2G_CAN_BASE_ID+2, .refresh_filter =
  // refresh_filter}; c2g_ins[2] = (C2G_t){.assemble_func = &c2g_pack3_load,
  // .pcan = &BOARDS_INTERACT_CAN_HANDLE, .stdid = C2G_CAN_BASE_ID+3,
  // .refresh_filter = refresh_filter};
  can_user_init(&BOARDS_INTERACT_CAN_HANDLE,
                (uint32_t[]){c2g_ins[0].stdid, 0, 0, 0}, 1);
}

/**
 * @brief 板间通信数据包发送
 */
void board_interact_send(void) {
  interact_dt = DWT_GetDeltaT(&interact_dwt_cnt);
  if (!interact_dt)
    interact_dt = INTERACT_TIM_FREQ;
#if MACHINE_TYPE == GIMBAL_MASTER
  for (uint8_t i = 0; i < G2C_NUM; i++) {
    uint8_t tx_msg[CAN_DATA_LEN] = {0};
    memcpy(tx_msg, &g2c_ins[i].buffer, 8);
    g2c_ins[i].assemble_func(&g2c_ins[i]);
    can_transmit_data(g2c_ins[i].pcan, g2c_ins[i].stdid, tx_msg, CAN_DATA_LEN,
                      0);
  }
#elif MACHINE_TYPE == CHASSIS_SLAVE
  for (uint8_t i = 0; i < C2G_NUM; i++) {
    uint8_t tx_msg[CAN_DATA_LEN] = {0};
    memcpy(tx_msg, &c2g_ins[i].buffer, 8);
    c2g_ins[i].assemble_func(&c2g_ins[i]);
    can_transmit_data(c2g_ins[i].pcan, c2g_ins[i].stdid, tx_msg, CAN_DATA_LEN,
                      0);
  }
#endif
}
/**
 * @brief 板间通信数据包解析
 */
void board_interact_parse(uint8_t *buffer, uint32_t stdid) {
#if MACHINE_TYPE == GIMBAL_MASTER
  for (uint8_t i = 0; i < C2G_NUM; i++) {
    if (stdid == c2g_ins[i].stdid) {
      c2g_ins[i].parse_func(buffer);
      c2g_ins[i].refresh_cnt++;
    }
  }
#elif MACHINE_TYPE == CHASSIS_SLAVE
  for (uint8_t i = 0; i < G2C_NUM; i++) {
    if (stdid == g2c_ins[i].stdid) {
      g2c_ins[i].parse_func(buffer);
      g2c_ins[i].refresh_cnt++;
    }
  }
#endif
}

/**
 * @brief	检测板间通信数据包异常情况
 */
void update_boards_interact_status(void) {
#define update_pack_freq(pack, dt)                                             \
  ;                                                                            \
  pack.refresh_freq =                                                          \
      LPF_update(&pack.refresh_filter, (pack.refresh_cnt) / dt);               \
  pack.refresh_cnt = 0
  // 反馈频率太低即出现异常
#if (BOARDS_MODE && MACHINE_TYPE == GIMBAL_MASTER)
  for (uint8_t i = 0; i < C2G_NUM; i++) {
    update_pack_freq(c2g_ins[i], 1.f / C2G_INTERACT_TIM_FREQ);
  }
#elif (BOARDS_MODE && MACHINE_TYPE == CHASSIS_SLAVE)
  for (uint8_t i = 0; i < G2C_NUM; i++) {
    update_pack_freq(g2c_ins[i], 1.f / G2C_INTERACT_TIM_FREQ);
  }
#endif
}
#endif
