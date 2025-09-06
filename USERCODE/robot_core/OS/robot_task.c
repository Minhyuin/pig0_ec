/**
 * @file robot_task.c
 * @author ttf
 * @brief 操作系统任务调度仪表盘
 *
 *
 * @copyright SCNU-PIONEER (c) 2024-2025
 *
 */
#include "robot_task.h"
#include "APP/robot.h"
#include "self_delay.h"

#define TASK_NUM 10

/*互斥信号量*/
volatile xSemaphoreHandle
    xMutex_set_all_motor_output;             // 用于保护set_all_motor_output函数
volatile xSemaphoreHandle xMutex_rc_update;  // 用于保护遥控器数据
volatile xSemaphoreHandle xMutex_update_tag; // 保护update_tag函数

/**
 * @name 任务调度仪表盘
 * @attention 全局变量需要用volatile修饰 且确保对该变量的所有访问均为原子操作!!!
 * 或使用互斥信号量进行锁保护 例如:会调用 set_all_motor_output
 * 的函数在发生调用时只允许同一优先级的任务调用 否则需互斥信号量进行保护
 * 不然程序卡死!!!
 * @attention 尽量避免"优先级反转的问题"
 */

// 任务优先级 数字越大 优先级越大
#define START_TASK_PRIO 1
// 任务堆栈大小
#define START_STK_SIZE 128
// 任务句柄
TaskHandle_t StartTask_Handler;
// 任务函数
void start_task(void *pvParameters);

// 测试任务
#define TEST_TASK_PRIO 6
#define TEST_STK_SIZE 128
TaskHandle_t TEST_Task_Handler;
void TEST_task(void *pvParameters);

// INS更新任务
#define IMU_TASK_PRIO 7
#define IMU_STK_SIZE 128
TaskHandle_t IMU_Task_Handler;
void IMU_task(void *pvParameters);

// 状态机
#define STATE_TASK_PRIO 8
#define STATE_STK_SIZE 128
TaskHandle_t STATE_Task_Handler;
void STATE_task(void *pvParameters);

// 云台控制核心任务
#define GIMBAL_TASK_PRIO 6
#define GIMBAL_STK_SIZE 128
TaskHandle_t GIMBAL_Task_Handler;
void GIMBAL_task(void *pvParameters);

// 底盘控制核心任务
#define CHASSIS_TASK_PRIO 6
#define CHASSIS_STK_SIZE 256
TaskHandle_t CHASSIS_Task_Handler;
void CHASSIS_task(void *pvParameters);

// 发射机构控制核心任务
#define SHOOTER_TASK_PRIO 6
#define SHOOTER_STK_SIZE 128
TaskHandle_t SHOOTER_Task_Handler;
void SHOOTER_task(void *pvParameters);

// LED指示任务
#define RGB_TASK_PRIO 8
#define RGB_STK_SIZE 128
TaskHandle_t RGB_Task_Handler;
void RGB_task(void *pvParameters);

// 蜂鸣器任务
#define BUZZER_TASK_PRIO 8
#define BUZZER_STK_SIZE 64
TaskHandle_t BUZZER_Task_Handler;
void BUZZER_task(void *pvParameters);

// 总线侦测任务
#define DETECT_TASK_PRIO 5
#define DETECT_STK_SIZE 128
TaskHandle_t DETECT_Task_Handler;
void DETECT_task(void *pvParameters);

// 裁判系统发送任务
#define REFEREE_TASK_PRIO 5
#define REFEREE_STK_SIZE 100
TaskHandle_t REFEREETask_Handler;
void REFEREE_task(void *pvParameters);

uint16_t task_stack_available[TASK_NUM] = {0};

void task_create(void) {
  // 创建开始任务
  xTaskCreate((TaskFunction_t)start_task,          // 任务函数
              (const char *)"start_task",          // 任务名称
              (uint16_t)START_STK_SIZE,            // 任务堆栈大小
              (void *)NULL,                        // 传递给任务函数的参数
              (UBaseType_t)START_TASK_PRIO,        // 任务优先级
              (TaskHandle_t *)&StartTask_Handler); // 任务句柄
  // 创建信号量
  xMutex_set_all_motor_output = xSemaphoreCreateMutex();
  xMutex_rc_update = xSemaphoreCreateMutex();
  xMutex_update_tag = xSemaphoreCreateMutex();

  // 开启任务调度
  vTaskStartScheduler();
}
// 开始任务的任务函数
void start_task(void *pvParameters) {
  taskENTER_CRITICAL(); // 进入临界区

#if USE_TEST_FILE
  xTaskCreate((TaskFunction_t)TEST_task, (const char *)"TEST_task",
              (uint16_t)TEST_STK_SIZE, (void *)NULL,
              (UBaseType_t)TEST_TASK_PRIO, (TaskHandle_t *)&TEST_Task_Handler);
#endif

  xTaskCreate((TaskFunction_t)STATE_task, (const char *)"STATE_task",
              (uint16_t)STATE_STK_SIZE, (void *)NULL,
              (UBaseType_t)STATE_TASK_PRIO,
              (TaskHandle_t *)&STATE_Task_Handler);

  xTaskCreate((TaskFunction_t)IMU_task, (const char *)"IMU_task",
              (uint16_t)IMU_STK_SIZE, (void *)NULL, (UBaseType_t)IMU_TASK_PRIO,
              (TaskHandle_t *)&IMU_Task_Handler);

  xTaskCreate((TaskFunction_t)RGB_task, (const char *)"RGB_task",
              (uint16_t)RGB_STK_SIZE, (void *)NULL, (UBaseType_t)RGB_TASK_PRIO,
              (TaskHandle_t *)&RGB_Task_Handler);

  xTaskCreate((TaskFunction_t)BUZZER_task, (const char *)"BUZZER_task",
              (uint16_t)BUZZER_STK_SIZE, (void *)NULL,
              (UBaseType_t)BUZZER_TASK_PRIO,
              (TaskHandle_t *)&BUZZER_Task_Handler);

  xTaskCreate((TaskFunction_t)GIMBAL_task, (const char *)"GIMBAL_task",
              (uint16_t)GIMBAL_STK_SIZE, (void *)NULL,
              (UBaseType_t)GIMBAL_TASK_PRIO,
              (TaskHandle_t *)&GIMBAL_Task_Handler);

  xTaskCreate((TaskFunction_t)CHASSIS_task, (const char *)"CHASSIS_task",
              (uint16_t)CHASSIS_STK_SIZE, (void *)NULL,
              (UBaseType_t)CHASSIS_TASK_PRIO,
              (TaskHandle_t *)&CHASSIS_Task_Handler);

  xTaskCreate((TaskFunction_t)SHOOTER_task, (const char *)"SHOOTER_task",
              (uint16_t)SHOOTER_STK_SIZE, (void *)NULL,
              (UBaseType_t)SHOOTER_TASK_PRIO,
              (TaskHandle_t *)&SHOOTER_Task_Handler);

  xTaskCreate((TaskFunction_t)DETECT_task, (const char *)"DETECT_task",
              (uint16_t)DETECT_STK_SIZE, (void *)NULL,
              (UBaseType_t)DETECT_TASK_PRIO,
              (TaskHandle_t *)&DETECT_Task_Handler);

  //		xTaskCreate((TaskFunction_t )UI_task,
  //                (const char*    )"UI_task",
  //                (uint16_t       )UI_STK_SIZE,
  //                (void*          )NULL,
  //                (UBaseType_t    )UI_TASK_PRIO,
  //                (TaskHandle_t*  )&UITask_Handler);

  vTaskDelete(StartTask_Handler); // 删除开始任务
  taskEXIT_CRITICAL();            // 退出临界区
}

// portTICK_RATE_MS 为1ms
TickType_t xTimeIncrement_t;
void IMU_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_imu_tim_cycle(); // T
    vTaskDelayUntil(&xLastWakeTime, (1000.f / IMU_TIM_FREQ) / portTICK_RATE_MS);
    task_stack_available[0] = uxTaskGetStackHighWaterMark(IMU_Task_Handler);
  }
}

void STATE_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_state_update(); // T
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / STATE_TIM_FREQ) / portTICK_RATE_MS);
    task_stack_available[1] = uxTaskGetStackHighWaterMark(STATE_Task_Handler);
  }
}

void GIMBAL_task(void *pvParameters) {
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_gimbal_ctrl(); // T
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / GIMBAL_TIM_FREQ) / portTICK_RATE_MS);
    task_stack_available[2] = uxTaskGetStackHighWaterMark(GIMBAL_Task_Handler);
  }
}

void CHASSIS_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_chassis_ctrl(); // T
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / CHASSIS_TIM_FREQ) / portTICK_RATE_MS); // 200Hz
    task_stack_available[4] = uxTaskGetStackHighWaterMark(CHASSIS_Task_Handler);
  }
}

void SHOOTER_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_shooter_ctrl(); // T
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / SHOOTER_TIM_FREQ) / portTICK_RATE_MS); // 200Hz
    task_stack_available[5] = uxTaskGetStackHighWaterMark(SHOOTER_Task_Handler);
  }
}

void RGB_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
#if defined(STM32H723xx)
    RGB_loop(10, 0.5, 1);
#elif defined(STM32F407xx)
    RGB_loop(10, 0.5);
#endif                                                                  // T
    vTaskDelayUntil(&xLastWakeTime, (1000.f / 8.f) / portTICK_RATE_MS); // 8Hz
    task_stack_available[3] = uxTaskGetStackHighWaterMark(RGB_Task_Handler);
  }
}

void BUZZER_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    buzzer_proc();                                                       // T
    vTaskDelayUntil(&xLastWakeTime, (1000.f / 10.f) / portTICK_RATE_MS); // 10Hz
    task_stack_available[3] = uxTaskGetStackHighWaterMark(BUZZER_Task_Handler);
  }
}

void TEST_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_test_loop(); // T
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / TEST_TIM_FREQ) / portTICK_RATE_MS); // 200Hz
    task_stack_available[TASK_NUM - 1] =
        uxTaskGetStackHighWaterMark(TEST_Task_Handler);
  }
}

void DETECT_task(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    robot_bus_detect(); // T
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / DETECT_TIM_FREQ) / portTICK_RATE_MS); // 10Hz
    task_stack_available[6] = uxTaskGetStackHighWaterMark(DETECT_Task_Handler);
  }
}

extern UART_HandleTypeDef REFEREE_UART_HANDLE;
void REFEREEask(void *pvParameters) {

  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime,
                    (1000.f / 200.f) / portTICK_RATE_MS); // 10Hz
  }
}
