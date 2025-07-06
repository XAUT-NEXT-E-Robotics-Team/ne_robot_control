/**
 * @file robot_task.h
 * @author imgzw
 * @brief  用于创建和管理机器人相关的任务，不使用STM32CubeMX的FreeRTOS配置，
 *         而是手动创建任务，便于管理
 * @attention 本头文件只能在robot.c中包含
 * @version 0.1
 * @date 2025-07-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "daemon.h"
#include "robot_cmd.h"

osThreadId daemonTaskHandle;


void StartDaemonTask(void const *argument);

/**
 * @brief 创建相关任务
 * @
 * 
 */
void RobotOSTaskCreate(void)
{
    osThreadDef(deamon, StartDaemonTask, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(deamon), NULL);
}


__attribute__((noreturn)) void StartDaemonTask(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(10);
    }  
}


