/**
 * @file robot_cmd.c
 * @author imgzw
 * @brief 机器人控制指令发送/接收/处理
 * @version 0.1
 * @date 2025-07-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "robot_cmd.h"

//module
#include "remote_control.h"


// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"



static FSI6Data_t *fs16data;

void RobotCmdInit(void)
{
    fs16data = FSI6RemoteControlInit(&huart3);
}
