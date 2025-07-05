#include "task_rtos.h"
#include "task_init.h"
#include "bsp_init.h"

void TaskInit(void)
{
    BSPInit();
    RobotOSTaskCreate();
    MotorTestInit();
    LOGINFO("[freeRTOS] Task Init Success");
}


