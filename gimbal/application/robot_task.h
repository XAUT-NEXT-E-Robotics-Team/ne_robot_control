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

#ifndef _ROBOT_TASH_H
#define _ROBOT_TASH_H

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "robot.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "daemon.h"
#include "robot_cmd.h"
#include "bsp_init.h"
#include "motor_task.h"
#include "ins_task.h"
#include "shoot.h"

osThreadId IMUTaskHandle;
osThreadId daemonTaskHandle;
osThreadId RobotTaskHandle;
osThreadId motorTaskHandle;
void StartIMUTask(void const *argument);
void StartDaemonTask(void const *argument);
void StartRoBotTask(void const *argument);
void StartMotorTask(void const *argument);
/**
 * @brief 创建相关任务
 * @
 * 
 */
void RobotOSTaskCreate(void)
{   
    osThreadDef(IMU, StartIMUTask, osPriorityRealtime, 0, 256);
    IMUTaskHandle = osThreadCreate(osThread(IMU), NULL);
    osThreadDef(deamon, StartDaemonTask, osPriorityBelowNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(deamon), NULL);
    osThreadDef(Robot,StartRoBotTask,osPriorityNormal,0,1024);
    RobotTaskHandle = osThreadCreate(osThread(Robot),NULL);
    osThreadDef(motortask,StartMotorTask,osPriorityAboveNormal,0,256);
    motorTaskHandle = osThreadCreate(osThread(motortask),NULL);
}

__attribute__((noreturn)) void StartIMUTask(void const *argument)
{
     float daemon_dt;
     float daemon_start;
    LOGINFO("[freeRTOS] IMU Task Start");
    for (;;)
    {
        // 1000Hz
        daemon_start = DWT_GetTimeline_ms();
        INS_Task();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 1)
            LOGERROR("[freeRTOS] IMU Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(1);
    }  
}


__attribute__((noreturn)) void StartDaemonTask(void const *argument)
{
     float daemon_dt;
     float daemon_start;
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

__attribute__((noreturn)) void StartRoBotTask(void const *argument)
{
  float RoBot_dt;
  float RoBot_start;
LOGINFO("[freeRTOS] RoBot Task Start");
for(;;){
        //500Hz
        RoBot_start = DWT_GetTimeline_ms();
        RobotTask() ;
        RoBot_dt = DWT_GetTimeline_ms() - RoBot_start;
        if(RoBot_dt>2)
         LOGERROR("[freeRTOS] chassis Task is being DELAY! dt = [%f]",&RoBot_dt);
         osDelay(1);
    }
}

__attribute__((noreturn)) void StartMotorTask(void const *argument)
{
     float motor_dt;
     float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(2);
    }  
} 




#endif


