/**
 * @file shoot.c
 * @author cherishQA
 * @brief 
 * @version 0.1
 * @date 2025-07-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "cmsis_os.h"

//define frileftmotor frirightmotor  loadermotor 
DJIMotorInstance *frileftmotor ,*frirightmotor ,*loadermotor ;
//define shoot subscriber for geting cmd date
Subscriber_t  *shoot_sub ;
//define shoot publisher for sending feedback date
Publisher_t   *shoot_pub ;

Shoot_Ctrol_Cmd_s shoot_cmd_recv ; //shoot cmd recive struct for 传参

void ShootInit()
{
  //frileftmotor frurightmotor  
  Motor_Init_Config_s  frictionmotor_config = {
   .can_init_config = {
    .can_handle = &hcan2 ,
   } ,
   .controller_param_init_config = {
    .speed_PID = {
        .Kp = 3.0f ,
        .Ki = 0.13f ,
        .Kd = 0.002f ,
        .IntegralLimit = 1000.0f ,
        .MaxOut = 16000.0f ,
        .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement),
    },
   },
   .controller_setting_init_config = {
    .speed_feedback_source = MOTOR_FEED , //feedback date from motor
		.outer_loop_type = SPEED_LOOP , 
    .close_loop_type = SPEED_LOOP , //choose cloop type
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL ,  //motor_run_deriction flag
   },
   .motor_type = M3508 , //choose motor type
  };
 //seting friction_motor
frictionmotor_config.can_init_config.tx_id = 1 ;
frileftmotor = DJIMotorInit(&frictionmotor_config);

frictionmotor_config.can_init_config.tx_id = 2 ;
frictionmotor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE ;  //反转
frirightmotor = DJIMotorInit(&frictionmotor_config);  
  
  //loadermotor
  Motor_Init_Config_s loadermotor_config ={
    .can_init_config = {
        .can_handle = &hcan1 ,
        .tx_id = 3 ,
    },                                //seting can inital
    .controller_param_init_config = {
        .speed_PID = {
            .Kp = 10.0f ,
            .Ki = 0.000f ,
            .Kd = 0.0f ,
            .IntegralLimit = 1000.0f ,
            .MaxOut = 13000.0f ,
            .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement),
        },
        .angle_PID = {
            .Kp = 0.3f ,
            .Ki = 0.00f ,
            .Kd = 0.0f ,
            .IntegralLimit = 2000.0f ,
            .MaxOut = 15000.0f ,
            .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement),
        },      
    },
    .controller_setting_init_config = { //loadermotor use angle loop 
        .speed_feedback_source = MOTOR_FEED ,
        .angle_feedback_source = MOTOR_FEED ,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL ,
        .outer_loop_type = ANGLE_LOOP ,
        .close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP) } ,
        
    .motor_type = M3508 };              //seting motor type

    loadermotor = DJIMotorInit(&loadermotor_config);    //seting loadermotor

//finish publisher register and subscriber register
shoot_sub = SubRegister("shoot_cmd",sizeof(Shoot_Ctrol_Cmd_s));
//shoot_pub = PubRegister("shootfeedback",sizeof());   //shoot部分没有其他要传回来的了，可以直接在keil中查出对应的电机状态

}


void ShootTask() 
{
    //get cmd date
   SubGetMessage(shoot_sub,&shoot_cmd_recv);
    //check shoot state 
   if(shoot_cmd_recv.shoot_mode == SHOOT_ON) //enable
   {
    DJIMotorEnable(frileftmotor);
    DJIMotorEnable(frirightmotor);   
    DJIMotorEnable(loadermotor);    
   }   
   else                                      //disable
   {                                        
    DJIMotorStop(frileftmotor);
    DJIMotorStop(frirightmotor);
    DJIMotorStop(loadermotor);
   }

   switch (shoot_cmd_recv.loader_mode)
   {
     //卡弹处理
     if(loadermotor->measure.real_current >= 9.5  ) {
      osDelay(200);  //0.2s
      if(loadermotor->measure.real_current >= 9.5 )  //电流长时间(0.2s)过高 ，确认卡弹 
      shoot_cmd_recv.loader_mode = LOAD_REVERSE ; //反转
      else   shoot_cmd_recv.loader_mode = LOAD_BURSTFIRE ;  //等2s如果恢复正常状态 ，继续来连续发射 
      }

      case LOAD_1_BULLET :               
        //shoot once (2.3连发可由操作手掌握，所以放在连发一起)
        DJIMotorOuterLoop(loadermotor,ANGLE_LOOP);
        DJIMotorSetRef(loadermotor,loadermotor->measure.total_angle + ONEBULLUTANGLE);
        //后面需要加时间（从DWT_GETTIM中获取时间优化操作手单发手感）
      break;
      case LOAD_BURSTFIRE :                  //shoot all the time 
        DJIMotorOuterLoop(loadermotor,SPEED_LOOP);  
        DJIMotorSetRef(loadermotor,shoot_cmd_recv.shoot_rate * (360/NUM_PER_CIRCLE) * LOADEMOTOR_JSB);
        //计算方式：目标射频 * 角速度（360/拨盘一圈的弹丸数量[及拨出一发弹丸需要的角速度]） * 减速比（小齿轮带动大齿轮）
      break;
      case LOAD_REVERSE :                    //RUN REVERSE(反转)
        DJIMotorOuterLoop(loadermotor,SPEED_LOOP);
        DJIMotorSetRef(loadermotor,-((360/NUM_PER_CIRCLE)*LOADEMOTOR_JSB));  //以1hz的射频退弹
      break;
      case LOAD_STOP :
        DJIMotorOuterLoop(loadermotor,SPEED_LOOP);
        DJIMotorSetRef(loadermotor,0);         //enable stop (有力stop) 
      break;
      default :                                //other situation
      break ;
    }

    //FRICTION_MOTOR  
if(shoot_cmd_recv.friction_mode == FRICTION_ON)
{
  switch(shoot_cmd_recv.Blluet_speed)
  {
  case BULLET_SPEED1:                       //friction    speed = 15  (这个的实际去测一下)
    DJIMotorSetRef(frileftmotor,6000);      //(set speed :5500~7000)
    DJIMotorSetRef(frirightmotor,6000);
    break;
  case BULLET_SPEED2:                       //friction    speed = 18 
    DJIMotorSetRef(frileftmotor,6000);      //(set speed :5500~7000)
    DJIMotorSetRef(frirightmotor,6000); 
    break;
  default:
    DJIMotorSetRef(frileftmotor,-12000);     //(set speed :5500~7000)
    DJIMotorSetRef(frirightmotor,12000);
    break;
  }

}
else 
{   // friction   speed = 0
    DJIMotorSetRef(frileftmotor,0);
    DJIMotorSetRef(frirightmotor,0);
}

//if need to send feedback date (use PubPushMessage)

}
