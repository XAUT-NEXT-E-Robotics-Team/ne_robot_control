/**
 * @file shoot.c
 * @author cherishQA
 * @brief 
 * @version 0.1
 * @date 2025-07-9
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "gimbal.h"
#include "dji_motor.h"       
#include "message_center.h"   //消息中心
#include "general_def.h"      //tools
#include "robot_def.h"
#include "ins_task.h"         //姿态解算
#include "vofa.h"     
#include "usart.h"

attitude_t *gimba_IMU_date ;  //云台IMU数据

DJIMotorInstance *MOTOR_YAW  , *MOTOR_PITCH  ;  //define two motors (for gimbal run)
Publisher_t *gimbal_pub ;                       //define gimbal publisher (for send date)  
Subscriber_t *gimbal_sub ;                      //define gimbal subscriber (for recive date)

Gimbal_Ctrol_Cmd_s  gimbal_cmd_recv;            //define gimbal_cmd_control  struct (for recive controldate from CMD)
Gimbal_Upload_Date_s gimbal_feedback_date;      //define gimbal_update struct ( gimbal motor feedback dates 回传给 CMD)


 void GimbalInit()
 {
   //初始化IMU
   gimba_IMU_date = INS_Init(); //IMU初始化，获取姿态数据指针赋值给yaw电机
  
   //YAW
   Motor_Init_Config_s yaw_config = {
       .can_init_config = {
       .can_handle = &hcan1,
       .tx_id = 3 ,           //send ID
      },
      .controller_param_init_config = {
      .angle_PID = {
        .Kp = 1.5f,
        .Ki = 0.1f,
        .Kd = 0.0f,
        .IntegralLimit = 0.1f,  //积分限幅
        .MaxOut = 6.0f ,        //输出限幅                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        .DeadBand = 0.0 ,       //死区
        .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_DerivativeFilter), //梯形积分|积分限幅|微分先行
      },
      .speed_PID = {
        .Kp = -1500.0f,
        .Ki = 0.0f,
        .Kd = -0.01f,
        .IntegralLimit = 0.0f,  //
        .MaxOut = 15000.0f,
        .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_DerivativeFilter), //梯形积分|积分限幅|微分先行
      },
			.other_angle_feedback_ptr = &gimba_IMU_date->YawTotalAngle, //PID参考值，也称实际反馈的值
     //其他的反馈值     
      .other_speed_feedback_ptr = &gimba_IMU_date->Gyro[2], //BMIO88角速度
       },
      .controller_setting_init_config = {
        .angle_feedback_source =    OTHER_FEED ,        // MOTOR_FEED|OTHER_FEED :使用后者需要指定数据来源
        .speed_feedback_source =    OTHER_FEED ,         // speed 用BMI088中反馈的角速度
        .outer_loop_type = ANGLE_LOOP ,
        .close_loop_type =(Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP) , //速度环与角度环 
        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE , //电机正转标志
         },
      .motor_type = GM6020 };  //电机类型
    //PITCH
    Motor_Init_Config_s pitch_config = {
       .can_init_config = {
        .can_handle = &hcan1 ,     //can_handle_type
        .tx_id = 2				 ,       //send ID
       },
       .controller_param_init_config ={
        .angle_PID = {
          .Kp = 1.0f , 
          .Ki = 0.00f ,
          .Kd = 0.05f ,
          .IntegralLimit = 0.00f ,
          .MaxOut = 8.0f ,
          .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_DerivativeFilter), //梯形积分|微分先行
        },
        .speed_PID = {
          .Kp = 1000.0f ,
          .Ki = 0.0f ,
          .Kd = 10.0f ,
          .IntegralLimit = 0.0f ,
          .MaxOut = 15000.0f ,
          .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_DerivativeFilter), //梯形积分|微分先行
        },
       //其他的反馈值     
        .other_angle_feedback_ptr = &gimba_IMU_date->Roll ,
        .other_speed_feedback_ptr = &gimba_IMU_date->Gyro[1] ,

       },
       .controller_setting_init_config = {
        .angle_feedback_source = OTHER_FEED , //PID测量值的选择，如果用MOTOR_FEED就是使用电机的编码器
        .speed_feedback_source = OTHER_FEED ,
        .outer_loop_type = ANGLE_LOOP ,
        .close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP) ,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL ,
       },
       .motor_type = GM6020 }; 
      //DJI_motor
      MOTOR_YAW = DJIMotorInit(&yaw_config);
      MOTOR_PITCH = DJIMotorInit(&pitch_config);

      //GIMBAL pubulisher and subscriber register
      gimbal_pub = PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Date_s));
      gimbal_sub = SubRegister("gimbal_cmd",sizeof(Gimbal_Ctrol_Cmd_s));
 }

void GimbalTask()
{

  //GET CMD DATE
  SubGetMessage(gimbal_sub,&gimbal_cmd_recv); 

  //MODE TO CHOOSE
  switch (gimbal_cmd_recv.gimbal_mode)
  {  //云台无力
  case  GIMBAL_STOP  :
    DJIMotorStop(MOTOR_YAW);
    DJIMotorStop(MOTOR_PITCH);
    break;
     //云台跟随底盘模式
  case GIMBAL_GYRO_MODE :             //   GIMBAL_GYRO_MODE :
    DJIMotorEnable(MOTOR_YAW);        //enable YAW and PITCH
    DJIMotorEnable(MOTOR_PITCH);
    //这里是更改PID的测量值，不需要更换来源了
    DJIMotorChangeFeed(MOTOR_YAW,ANGLE_LOOP,OTHER_FEED);   //change feedback frome
    DJIMotorChangeFeed(MOTOR_YAW,SPEED_LOOP,OTHER_FEED);
    DJIMotorChangeFeed(MOTOR_PITCH,ANGLE_LOOP,OTHER_FEED);
    DJIMotorChangeFeed(MOTOR_PITCH,SPEED_LOOP,OTHER_FEED);
    DJIMotorSetRef(MOTOR_YAW,gimbal_cmd_recv.yaw);         //seting djimotor 
    //DJIMotorSetRef(MOTOR_PITCH,gimbal_cmd_recv.pitch);
    break;  
  default:
    break;
  }
  
  //设置反馈数据,主要是imu和yaw的ecd  
  gimbal_feedback_date.gimbal_imu_date =  *gimba_IMU_date;
  gimbal_feedback_date.yaw_motor_single_round_angle = MOTOR_YAW->measure.angle_single_round;
  
  
  //gimbal publisher  send  feedback date
  PubPushMessage(gimbal_pub,(void *)&gimbal_feedback_date);


} 
