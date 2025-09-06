#ifndef _TEST_H
#define _TEST_H
#include "APP/robot.h"
#include "Base/rc.h"


void motor_test_init(void);
HAL_StatusTypeDef motor_test_loop(float dt, robot_t *robot);

#endif // !_TEST_H
