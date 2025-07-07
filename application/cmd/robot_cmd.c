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
#include "robot_def.h"
#include "message_center.h"
// module
#include "remote_control.h"

// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#define STICK_TO_SPEED_RATIO 0.02f // 摇杆到速度的比例系数

Publisher_t *chassis_cmd_pub;  // 地盘控制指令发布者
Subscriber_t *chassis_cmd_sub; // 地盘控制指令订阅者
FSI6Data_t *fs16data;
Chassis_Ctrl_Cmd_s chassis_cmd_send; // 地盘控制指令发送结构体
Robot_Status_e robot_state;          // robot整体工作状态

void RobotCmdInit(void)
{
    fs16data = FSI6RemoteControlInit(&huart3);
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s)); // 地盘控制指令发布者
    robot_state = ROBOT_READY;                                                // robot进入准备状态
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RoBotCmdRemoteControlSet(void)
{
    // 使能底盘模式
    if (fs16data->SC_CH7 == RC_SW_MID)
    { 
        chassis_cmd_send.chassis_mode = chassis_unfollow;
    }
    chassis_cmd_send.VX = STICK_TO_SPEED_RATIO * (fs16data->L_UD); // 前后平移
    chassis_cmd_send.VY = STICK_TO_SPEED_RATIO * (fs16data->L_LR); // 左右平移
    if (fs16data->SC_CH7 == RC_SW_DOWN && fs16data->V2_R > 200)
    {                                            // 使能小陀螺
        chassis_cmd_send.WZ = (fs16data->V2_R)-RC_KNOB_MIN; // 由V2映射小陀螺的速度,并建立死区防止误触
    }
}

/* ROBOT核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RoBotCmdTask(void)
{
    // ROBOTcontrolSet
    RoBotCmdRemoteControlSet();
    // send chassis_cmd
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
}
