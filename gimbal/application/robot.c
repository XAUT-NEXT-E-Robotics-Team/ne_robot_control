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


extern TIM_HandleTypeDef htim4;

//蜂鸣器
void BuzzerRun( uint8_t BuzzerRun_count)
{

  for(uint8_t i = 0; i < BuzzerRun_count; i++)
  {
   __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 200);
   osDelay(100);//0.1S
   __HAL_TIM_SetCompare(&htim4 ,TIM_CHANNEL_3, 0);
   osDelay(50);//0.1S
  }
}

void Buzzer_Init( void )
{
 //使能tim时钟  
 HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); 
}


void robot_init(void)
{
    BSPInit();
    RobotCmdInit();
    RobotOSTaskCreate();
    GimbalInit();
    ShootInit();
	  Buzzer_Init();
    LOGINFO("[robot] Robot Init Success");
}

//ROBOT底盘，云台，发射 ，控制任务
void RobotTask(void)
{
    static  uint8_t buzzer_flag = 0;
    if(buzzer_flag == 0){
    BuzzerRun(2);
    buzzer_flag = 1 ; }

    RoBotCmdTask();
    GimbalTask();
    ShootTask();
}
