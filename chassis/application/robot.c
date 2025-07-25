/**
 * @file robot.c
 * @author imgzw
 * @brief 
 * @version 0.1
 * @date 2025-07-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "robot.h"
#include "robot_task.h"
void robot_init(void)
{
    BSPInit();
    ChassisInit();
    //@todo:是否需要关闭所有中断
    //创建/启用任务，需在初始化之后
    RobotOSTaskCreate();

    LOGINFO("[robot] Robot Init Success");
}

//ROBOT底盘，云台，发射 ，控制任务
void RobotTask(void)
{
    ChassisTask();
}


