#pragma once
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "motor_control.h"
#include "daemon.h"

osThreadId motorTestHandle;

void MotorTestTask(void const *argument);
void DEAmonTask(void const *argument);
void MotorControlTask(void const *argument);


/**
 * @brief 创建相关任务
 * 
 */
void RobotOSTaskCreate(void)
{
    osThreadDef(motorTest, MotorTestTask, osPriorityNormal, 0, 256);
    motorTestHandle = osThreadCreate(osThread(motorTest), NULL);

    osThreadDef(deamon, DEAmonTask, osPriorityNormal, 0, 128);
    osThreadCreate(osThread(deamon), NULL);

    osThreadDef(motorControl, MotorControlTask, osPriorityNormal, 0, 256);
    osThreadCreate(osThread(motorControl), NULL);
}


__attribute__((noreturn)) void MotorTestTask(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {  
        // 500Hz
        motor_start = DWT_GetTimeline_ms();
        DJIMotorControl();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 2)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(2);
    }
}

__attribute__((noreturn)) void DEAmonTask(void const *argument)
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

__attribute__((noreturn)) void MotorControlTask(void const *argument)
{
    static float motor_control_dt;
    static float motor_control_start;
    LOGINFO("[freeRTOS] Motor Control Task Start");
    for (;;)
    {
        // 100Hz
        motor_control_start = DWT_GetTimeline_ms();
        MotorTestControl();
        motor_control_dt = DWT_GetTimeline_ms() - motor_control_start;
        if (motor_control_dt > 10)
            LOGERROR("[freeRTOS] Motor Control Task is being DELAY! dt = [%f]", &motor_control_dt);
        osDelay(10);
    }
}
