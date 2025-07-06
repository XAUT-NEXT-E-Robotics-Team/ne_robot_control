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
    RobotCmdInit();
    
    LOGINFO("[robot] Robot Init Success");
}
