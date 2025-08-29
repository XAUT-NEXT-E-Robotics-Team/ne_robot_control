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
#include "gimbal.h"
#include "shoot.h"
#include "robot_task.h"

uint8_t BuzzerRun_count = 0 ;
extern TIM_HandleTypeDef htim4;


//蜂鸣器
void BuzzerRun()
{
 static uint16_t timeCount = 0 ;
 if( BuzzerRun_count > 0 )
 {
    timeCount += 2 ;
 }
 if( timeCount > 350 )
 {
    timeCount = 0 ;
    BuzzerRun_count-- ;
 }
 

   if (timeCount > 350 / 2) 
  {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  }
  else 
  {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
  }

}


void Buzzer_Init( uint8_t buzzerCount)
{
  BuzzerRun_count = buzzerCount ;  
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Stop(&htim4 , TIM_CHANNEL_3);
}


void robot_init(void)
{
    BSPInit();
    RobotCmdInit();
    RobotOSTaskCreate();
    GimbalInit();
    ShootInit();
    LOGINFO("[robot] Robot Init Success");

    Buzzer_Init(2);
    BuzzerRun();
}

//ROBOT底盘，云台，发射 ，控制任务
void RobotTask(void)
{
    RoBotCmdTask();
    GimbalTask();
    ShootTask();
}




