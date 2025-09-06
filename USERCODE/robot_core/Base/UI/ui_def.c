#include "ui_def.h"

#include "priority_ui.h"
#include "ui_protocol.h"

#define CLIENT_MID_P_X (1920 / 2)
#define CLIENT_MID_P_Y (1080 / 2)

uint16_t attack_point_x = CLIENT_MID_P_X;

// 尽量不要在靠近左下和右下画图,会被client界面的信息条和小地图等图层遮挡

ui_info_t dynamic_ui_info[DYNAMIC_UI_NUM] = {
    [SpdVel] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = CIRCLE,        // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 5,             // 图层数，0~9
            .ui_config.color = RED_BLUE,      // 颜色
            .ui_config.width = 5,             // 线条宽度
            .ui_config.start_x = 369,         // 起点 x 坐标
            .ui_config.start_y = 544,         // 起点 y 坐标
            .ui_config.radius = 19,           // 半径
        },
    [PitchAng] =
        {
            /*******不变配置*********/
            .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = CIRCLE,        // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 5,             // 图层数，0~9
            .ui_config.color = RED_BLUE,      // 颜色
            .ui_config.width = 5,             // 线条宽度
            .ui_config.start_x = 421,         // 起点 x 坐标
            .ui_config.start_y = 545,         // 起点 y 坐标
            .ui_config.radius = 8,            // 半径
        },
    // [Reload] =
    //     {
    //         /*******不变配置*********/
    //         .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    //         .ui_config.ui_type = CHAR,          // UI内容类型
    //         /*******可变配置*********/
    //         .ui_config.operate_type = MODIFY, // 操作类型
    //         .ui_config.layer = 1,             // 图层数，0~9
    //         .ui_config.color = GREEN,         // 颜色
    //         .ui_config.size = 40,             // 字体大小
    //         .ui_config.width = 2,             // 线条宽度
    //         .ui_config.start_x = 1039,        // 起点 x 坐标
    //         .ui_config.start_y = 482,         // 起点 y 坐标
    //         .ui_config.text = "R",
    //     },
    [VelNum] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = FLOAT,         // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 1,             // 图层数，0~9
            .ui_config.color = CYAN_BLUE,     // 颜色
            .ui_config.size = 15,             // 字体大小
            .ui_config.width = 2,             // 线条宽度
            .ui_config.start_x = 465,         // 起点 x 坐标
            .ui_config.start_y = 882,         // 起点 y 坐标
            .ui_config.float_num = 0.0f,      // 浮点数
            .ui_config.decimal = 1,           // 有效小数位数
        },
    [PitchNum] =
        {
            /*******不变配置*********/
            .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = FLOAT,         // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 1,             // 图层数，0~9
            .ui_config.color = GREEN,         // 颜色
            .ui_config.size = 15,             // 字体大小
            .ui_config.width = 2,             // 线条宽度
            .ui_config.start_x = 472,         // 起点 x 坐标
            .ui_config.start_y = 809,         // 起点 y 坐标
            .ui_config.float_num = 0.0f,      // 浮点数
            .ui_config.decimal = 4,           // 有效小数位数
        },
    [EnemyHP] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = LINE,          // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 6,             // 图层数，0~9
            .ui_config.color = RED_BLUE,      // 颜色
            .ui_config.width = 20,            // 线条宽度
            .ui_config.start_x = 959,         // 起点 x 坐标
            .ui_config.start_y = 870,         // 起点 y 坐标
            .ui_config.end_x = 961,           // 终点 x 坐标
            .ui_config.end_y = 870,           // 终点 y 坐标
        },
    [CapEnergy] =
        {
            /*******不变配置*********/
            .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = ARC,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY,                  // 操作类型
            .ui_config.layer = 2,                              // 图层数，0~9
            .ui_config.color = WHITE,                          // 颜色
            .ui_config.width = 20,                             // 线条宽度
            .ui_config.start_x = 1108,                         // 圆心 x 坐标
            .ui_config.start_y = 540,                          // 圆心 y 坐标
            .ui_config.end_x = 425,                            // x 半轴长度
            .ui_config.end_y = 425,                            // y 半轴长度
            .ui_config.start_angel = CAP_ENERGY_ARC_START_ENG, // 起始角度
            .ui_config.end_angel = CAP_ENERGY_ARC_END_ENG,     // 终止角度
        },
    [FOV] =
        {
            /*******不变配置*********/
            .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = RECTANGEL,      // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 2,             // 图层数，0~9
            .ui_config.color = WHITE,         // 颜色
            .ui_config.width = 1,             // 线条宽度
            .ui_config.start_x = 730,         // 起点(左下) x 坐标
            .ui_config.start_y = 350,         // 起点(左下) y 坐标
            .ui_config.end_x = 1260,          // 终点(右上) x 坐标
            .ui_config.end_y = 748,           // 终点(右上) y 坐标
            .ui_config.start_angel = 60,      // 起始角度
            .ui_config.end_angel = 120,       // 终止角度
        },
    [GimbalState] =
        {
            /*******不变配置*********/
            .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = CHAR,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 3,             // 图层数，0~9
            .ui_config.color = ORANGE,        // 颜色
            .ui_config.size = 20,             // 字体大小
            .ui_config.width = 2,             // 线条宽度
            .ui_config.start_x = 140,         // 起点 x 坐标
            .ui_config.start_y = 800,         // 起点 y 坐标
            .ui_config.text = "STOP",
        },
    [ChassisState] =
        {
            /*******不变配置*********/
            .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = CHAR,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 3,             // 图层数，0~9
            .ui_config.color = ORANGE,        // 颜色
            .ui_config.size = 20,             // 字体大小
            .ui_config.width = 2,             // 线条宽度
            .ui_config.start_x = 140,         // 起点 x 坐标
            .ui_config.start_y = 840,         // 起点 y 坐标
            .ui_config.text = "STOP",
        },
    [ShooterState] =
        {
            /*******不变配置*********/
            .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = CHAR,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 3,             // 图层数，0~9
            .ui_config.color = ORANGE,        // 颜色
            .ui_config.size = 20,             // 字体大小
            .ui_config.width = 2,             // 线条宽度
            .ui_config.start_x = 140,         // 起点 x 坐标
            .ui_config.start_y = 760,         // 起点 y 坐标
            .ui_config.text = "STOP",
        },
    [VisionState] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = CHAR,          // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 3,             // 图层数，0~9
            .ui_config.color = PINK,          // 颜色
            .ui_config.size = 30,             // 字体大小
            .ui_config.width = 2,             // 线条宽度
            .ui_config.start_x = 200 - 62,    // 起点 x 坐标
            .ui_config.start_y = 720,         // 起点 y 坐标
            .ui_config.text = "X",
        },
    [ChassisEnergy] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = ARC,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 3,             // 图层数，0~9
            .ui_config.color = ORANGE,        // 颜色
            .ui_config.width = 20,            // 线条宽度
            .ui_config.start_x = 1286,        // 圆心 x 坐标
            .ui_config.start_y = 540,         // 圆心 y 坐标
            .ui_config.end_x = 203,           // x 半轴长度
            .ui_config.end_y = 203,           // y 半轴长度
            .ui_config.start_angel = CHASSIS_ENERGY_ARC_START_ENG, // 起始角度
            .ui_config.end_angel = CHASSIS_ENERGY_ARC_END_ENG,     // 终止角度
        },
    [FricSwitch] =
        {
            /*******不变配置*********/
            .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = LINE,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY, // 操作类型
            .ui_config.layer = 3,             // 图层数，0~9
            .ui_config.color = RED_BLUE,      // 颜色
            .ui_config.width = 8,             // 线条宽度
            .ui_config.start_x = 906,         // 起点 x 坐标
            .ui_config.start_y = 485,         // 起点 y 坐标
            .ui_config.end_x = 1014,          // 终点 x 坐标
            .ui_config.end_y = 594,           // 终点 y 坐标
        },
    [LeftMove] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = LINE,          // UI内容类型
                                                /*******可变配置*********/
            .ui_config.operate_type = MODIFY,   // 操作类型
            .ui_config.layer = 4,               // 图层数，0~9
            .ui_config.color = GREEN,           // 颜色
            .ui_config.width = 3,               // 线条宽度
            .ui_config.start_x = DOWN_MOVE_LEFT_POS_START_X, // 起点 x 坐标
            .ui_config.start_y = DOWN_MOVE_LEFT_POS_START_Y, // 起点 y 坐标
            .ui_config.end_x = DOWN_MOVE_LEFT_POS_END_X,     // 终点 x 坐标
            .ui_config.end_y = DOWN_MOVE_LEFT_POS_END_Y,     // 终点 y 坐标
        },
    [RightMove] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = LINE,          // UI内容类型
                                                /*******可变配置*********/
            .ui_config.operate_type = MODIFY,   // 操作类型
            .ui_config.layer = 4,               // 图层数，0~9
            .ui_config.color = GREEN,           // 颜色
            .ui_config.width = 3,               // 线条宽度
            .ui_config.start_x = DOWN_MOVE_RIGHT_POS_START_X, // 起点 x 坐标
            .ui_config.start_y = DOWN_MOVE_RIGHT_POS_START_Y, // 起点 y 坐标
            .ui_config.end_x = DOWN_MOVE_RIGHT_POS_END_X,     // 终点 x 坐标
            .ui_config.end_y = DOWN_MOVE_RIGHT_POS_END_Y,     // 终点 y 坐标
        },
    [SpinArc] =
        {
            /*******不变配置*********/
            .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
            .ui_config.ui_type = ARC,           // UI内容类型
            /*******可变配置*********/
            .ui_config.operate_type = MODIFY,       // 操作类型
            .ui_config.layer = 2,                   // 图层数，0~9
            .ui_config.color = YELLOW,              // 颜色
            .ui_config.width = 5,                   // 线条宽度
            .ui_config.start_x = 960,               // 圆心 x 坐标
            .ui_config.start_y = 540,               // 圆心 y 坐标
            .ui_config.end_x = 300,                 // x 半轴长度
            .ui_config.end_y = 300,                 // y 半轴长度
            .ui_config.start_angel = 0,             // 起始角度
            .ui_config.end_angel = UI_SPINNING_DEG, // 终止角度
        },
};

ui_info_t static_ui_info[STATIC_UI_NUM] = {
    [PowerBottomLine] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = LINE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 0,      // 图层数，0~9
            .ui_config.color = WHITE,  // 颜色
            .ui_config.width = 8,      // 线条宽度
            .ui_config.start_x = 1487, // 起点 x 坐标
            .ui_config.start_y = 329,  // 起点 y 坐标
            .ui_config.end_x = 1454,   // 终点 x 坐标
            .ui_config.end_y = 352,    // 终点 y 坐标
        },
    [ChassisBottomLine] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = LINE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 0,      // 图层数，0~9
            .ui_config.color = WHITE,  // 颜色
            .ui_config.width = 8,      // 线条宽度
            .ui_config.start_x = 1476, // 起点 x 坐标
            .ui_config.start_y = 433,  // 起点 y 坐标
            .ui_config.end_x = 1440,   // 终点 x 坐标
            .ui_config.end_y = 452,    // 终点 y 坐标
        },
    // [PowerTopLine] =
    //     {
    //         /*******不变配置*********/
    //         .ui_config.ui_type = LINE, // UI内容类型
    //         /*******可变配置*********/
    //         .ui_config.layer = 0,      // 图层数，0~9
    //         .ui_config.color = WHITE,  // 颜色
    //         .ui_config.width = 8,      // 线条宽度
    //         .ui_config.start_x = 1503, // 起点 x 坐标
    //         .ui_config.start_y = 736,  // 起点 y 坐标
    //         .ui_config.end_x = 1470,   // 终点 x 坐标
    //         .ui_config.end_y = 713,    // 终点 y 坐标
    //     },
    // [ChassisTopLine] =
    //     {
    //         /*******不变配置*********/
    //         .ui_config.ui_type = LINE, // UI内容类型
    //         /*******可变配置*********/
    //         .ui_config.layer = 0,      // 图层数，0~9
    //         .ui_config.color = WHITE,  // 颜色
    //         .ui_config.width = 8,      // 线条宽度
    //         .ui_config.start_x = 1483, // 起点 x 坐标
    //         .ui_config.start_y = 636,  // 起点 y 坐标
    //         .ui_config.end_x = 1449,   // 终点 x 坐标
    //         .ui_config.end_y = 617,    // 终点 y 坐标
    //     },
    [RealAttVerticalLine] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = LINE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 5,               // 图层数，0~9
            .ui_config.color = GREEN,           // 颜色
            .ui_config.width = 1,               // 线条宽度
            .ui_config.start_x = REAL_AIM_X,    // 起点 x 坐标
            .ui_config.start_y = 540,           // 起点 y 坐标
            .ui_config.end_x = REAL_AIM_X,      // 终点 x 坐标
            .ui_config.end_y = 540 - 100 - 300, // 终点 y 坐标
        },
    [RealAttHorizontalLine] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CIRCLE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 5,            // 图层数，0~9
            .ui_config.color = RED_BLUE,     // 颜色
            .ui_config.width = 10,           // 线条宽度
            .ui_config.start_x = REAL_AIM_X, // 起点 x 坐标
            .ui_config.start_y = 540,        // 起点 y 坐标
            .ui_config.radius = 3,           // 半径
        },
    [Gimbal] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CHAR, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 4,     // 图层数，0~9
            .ui_config.color = GREEN, // 颜色
            .ui_config.size = 20,     // 字体大小
            .ui_config.width = 2,     // 线条宽度
            .ui_config.start_x = 76,  // 起点 x 坐标
            .ui_config.start_y = 800, // 起点 y 坐标
            .ui_config.text = "G:",
        },
    [Chassis] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CHAR, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 4,     // 图层数，0~9
            .ui_config.color = GREEN, // 颜色
            .ui_config.size = 20,     // 字体大小
            .ui_config.width = 2,     // 线条宽度
            .ui_config.start_x = 76,  // 起点 x 坐标
            .ui_config.start_y = 840, // 起点 y 坐标
            .ui_config.text = "C:",
        },
    [Shooter] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CHAR, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 4,     // 图层数，0~9
            .ui_config.color = GREEN, // 颜色
            .ui_config.size = 20,     // 字体大小
            .ui_config.width = 2,     // 线条宽度
            .ui_config.start_x = 76,  // 起点 x 坐标
            .ui_config.start_y = 760, // 起点 y 坐标
            .ui_config.text = "S:",
        },
    [Vision] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CHAR, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 4,         // 图层数，0~9
            .ui_config.color = CYAN_BLUE, // 颜色
            .ui_config.size = 20,         // 字体大小
            .ui_config.width = 2,         // 线条宽度
            .ui_config.start_x = 76,      // 起点 x 坐标
            .ui_config.start_y = 720,     // 起点 y 坐标
            .ui_config.text = "VISION",
        },
    [ValRefArc] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = ARC, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 3,         // 图层数，0~9
            .ui_config.color = CYAN_BLUE, // 颜色
            .ui_config.width = 3,         // 线条宽度
            .ui_config.start_x = 870,     // 圆心 x 坐标
            .ui_config.start_y = 540,     // 圆心 y 坐标
            .ui_config.end_x = 500,       // x 半轴长度
            .ui_config.end_y = 500,       // y 半轴长度
            .ui_config.start_angel = 230, // 起始角度
            .ui_config.end_angel = 310,   // 终止角度
        },
    [PitchArc] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = ARC, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 3,         // 图层数，0~9
            .ui_config.color = GREEN,     // 颜色
            .ui_config.width = 3,         // 线条宽度
            .ui_config.start_x = 900,     // 圆心 x 坐标
            .ui_config.start_y = 540,     // 圆心 y 坐标
            .ui_config.end_x = 480,       // x 半轴长度
            .ui_config.end_y = 480,       // y 半轴长度
            .ui_config.start_angel = 240, // 起始角度
            .ui_config.end_angel = 300,   // 终止角度
        },
    [BottomLine1] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = LINE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 5,                 // 图层数，0~9
            .ui_config.color = GREEN,             // 颜色
            .ui_config.width = 1,                 // 线条宽度
            .ui_config.start_x = REAL_AIM_X - 40, // 起点 x 坐标
            .ui_config.start_y = 540 - 80,        // 起点 y 坐标
            .ui_config.end_x = REAL_AIM_X + 40,   // 终点 x 坐标
            .ui_config.end_y = 540 - 80,          // 终点 y 坐标
        },
    [BottomLine3] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = LINE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 5,                 // 图层数，0~9
            .ui_config.color = GREEN,             // 颜色
            .ui_config.width = 1,                 // 线条宽度
            .ui_config.start_x = REAL_AIM_X - 30, // 起点 x 坐标
            .ui_config.start_y = 540 - 90,        // 起点 y 坐标
            .ui_config.end_x = REAL_AIM_X + 30,   // 终点 x 坐标
            .ui_config.end_y = 540 - 90,          // 终点 y 坐标
        },
    [BottomLine5] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = LINE, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 5,                 // 图层数，0~9
            .ui_config.color = GREEN,             // 颜色
            .ui_config.width = 1,                 // 线条宽度
            .ui_config.start_x = REAL_AIM_X - 20, // 起点 x 坐标
            .ui_config.start_y = 540 - 100,       // 起点 y 坐标
            .ui_config.end_x = REAL_AIM_X + 20,   // 终点 x 坐标
            .ui_config.end_y = 540 - 100,         // 终点 y 坐标
        },
    [EN] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CHAR, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 4,      // 图层数，0~9
            .ui_config.color = ORANGE, // 颜色
            .ui_config.size = 20,      // 字体大小
            .ui_config.width = 2,      // 线条宽度
            .ui_config.start_x = 1436, // 起点 x 坐标
            .ui_config.start_y = 432,  // 起点 y 坐标
            .ui_config.text = "EN",
        },
    [PO] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = CHAR, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 4,      // 图层数，0~9
            .ui_config.color = YELLOW, // 颜色
            .ui_config.size = 20,      // 字体大小
            .ui_config.width = 2,      // 线条宽度
            .ui_config.start_x = 1439, // 起点 x 坐标
            .ui_config.start_y = 333,  // 起点 y 坐标
            .ui_config.text = "PO",
        },
    [EnergyStaticArc] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = ARC, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 0,        // 图层数，0~9
            .ui_config.color = WHITE,    // 颜色
            .ui_config.width = 5,        // 线条宽度
            .ui_config.start_x = 1296,   // 圆心 x 坐标
            .ui_config.start_y = 540,    // 圆心 y 坐标
            .ui_config.end_x = 206,      // x 半轴长度
            .ui_config.end_y = 206,      // y 半轴长度
            .ui_config.start_angel = 62, // 起始角度
            .ui_config.end_angel = 120,  // 终止角度
        },
    [PowerStaticArc] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = ARC, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 0,        // 图层数，0~9
            .ui_config.color = WHITE,    // 颜色
            .ui_config.width = 5,        // 线条宽度
            .ui_config.start_x = 1100,   // 圆心 x 坐标
            .ui_config.start_y = 540,    // 圆心 y 坐标
            .ui_config.end_x = 445,      // x 半轴长度
            .ui_config.end_y = 445,      // y 半轴长度
            .ui_config.start_angel = 64, // 起始角度
            .ui_config.end_angel = 119,  // 终止角度
        },
    [EnemyHPRect] =
        {
            /*******不变配置*********/
            .ui_config.ui_type = RECTANGEL, // UI内容类型
            /*******可变配置*********/
            .ui_config.layer = 5,           // 图层数，0~9
            .ui_config.color = WHITE,       // 颜色
            .ui_config.width = 1,           // 线条宽度
            .ui_config.start_x = 960 - 310, // 起点(左下) x 坐标
            .ui_config.start_y = 870 - 14,  // 起点(左下) y 坐标
            .ui_config.end_x = 960 + 310,   // 终点(右上) x 坐标
            .ui_config.end_y = 870 + 14,    // 终点(右上) y 坐标
        },
};

ui_info_t *get_dynamic_ui_info(void) { return dynamic_ui_info; }
uint32_t get_dynamic_ui_num(void) {
  return sizeof(dynamic_ui_info) / sizeof(ui_info_t);
}
ui_info_t *get_static_ui_info(void) { return static_ui_info; }
uint32_t get_static_ui_num(void) {
  return sizeof(static_ui_info) / sizeof(ui_info_t);
}
