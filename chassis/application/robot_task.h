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
#include "bsp_init.h"
#include "motor_task.h"
#include "chassis.h"
#include "referee.h"

osThreadId daemonTaskHandle;
osThreadId RobotTaskHandle;
osThreadId motorTaskHandle;
osThreadId refereeTaskHandle;

void StartDaemonTask(void const *argument);
void StartRoBotTask(void const *argument);
void StartMotorTask(void const *argument);
void StartRefereeTask(void const *argument);

/**
 * @brief 创建相关任务
 * @
 * 
 */
void RobotOSTaskCreate(void)
{
    osThreadDef(deamon, StartDaemonTask, osPriorityBelowNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(deamon), NULL);
    osThreadDef(Robot,StartRoBotTask,osPriorityAboveNormal,0,1024);
    RobotTaskHandle = osThreadCreate(osThread(Robot),NULL);
    osThreadDef(motortask,StartMotorTask,osPriorityNormal,0,256);
    motorTaskHandle = osThreadCreate(osThread(motortask),NULL);
    osThreadDef(refereetask,StartRefereeTask,osPriorityBelowNormal,0,256);
    refereeTaskHandle = osThreadCreate(osThread(refereetask),NULL);
}


__attribute__((noreturn)) void StartDaemonTask(void const *argument)
{   // 50Hz
    static float daemon_dt;
    static float daemon_start;
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 20)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(20);
    }  
}


__attribute__((noreturn)) void StartRoBotTask(void const *argument)
{    //200Hz
    static float RoBot_dt;
    static float RoBot_start;
        LOGINFO("[freeRTOS] RoBot Task Start");
        for(;;){
        RoBot_start = DWT_GetTimeline_ms();
        RobotTask() ;
        RoBot_dt = DWT_GetTimeline_ms() - RoBot_start;
        if(RoBot_dt>5)
         LOGERROR("[freeRTOS] chassis Task is being DELAY! dt = [%f]",&RoBot_dt);
         osDelay(5);
    }
}


__attribute__((noreturn)) void StartMotorTask(void const *argument)
{   //1000HZ
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }  
} 

__attribute__((noreturn)) void StartRefereeTask(void const *argument)
{   //100Hz
    static float referee_dt;
    static float referee_start;
    LOGINFO("[freeRTOS] Referee Task Start");
    for (;;)
    {
        referee_start = DWT_GetTimeline_ms();
        referee_unpack_fifo_data();
        referee_dt = DWT_GetTimeline_ms() - referee_start;
        if (referee_dt > 10)
            LOGERROR("[freeRTOS] Referee Task is being DELAY! dt = [%f]", &referee_dt);
        osDelay(10);
    }  
}




#endif


