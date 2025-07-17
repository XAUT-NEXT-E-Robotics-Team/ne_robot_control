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
#include "dji_motor.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

//双机通信
#include "can_comm.h"
CANCommInstance *cmd_can_comm ;

#define YAW_ALING_ANGLE        (YAW_CHASSIS_ANGLE_ECD * ECD_ANGLE_COEF_DJI)   //对齐时的yaw角度  （0~360）
#define PITCH_HORIZON_ANGLE    (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)    //对齐时的pitch角度  （0~360）

//FSI6
float STICK_TO_SPEED_CHASSIS = 0.01f; // 摇杆到速度的比例系数(CHASSIS)（最大速度是 7.84 m/s）
float STICK_TO_SPEED_GIMBAL =  0.01f; // 摇杆到速度的比例系数(GIMBAL)
FSI6Data_t *fs16data;

//CHASSIS_CMD
Chassis_Ctrl_Cmd_s chassis_cmd_send; // 地盘控制指令发送结构体
Chassis_Upload_Data_s chassis_feedback_date; // 底盘feedback date 回传

//GIMBAL_CMD
Publisher_t *gimbal_cmd_pub;  //cmd send publisher insatance  
Subscriber_t *gimbal_cmd_sub; //feedback subscriber insatance
Gimbal_Ctrol_Cmd_s gimbal_cmd_send;  //云台控制指令发送结构体
Gimbal_Upload_Date_s gimbal_feedback; //云台feedback date 回传

//SHOOT_CMD
Publisher_t *shoot_cmd_pub ;  //cmd send publisher instance
Subscriber_t *shoot_cmd_sub ; //feedback subscriber instance
Shoot_Ctrol_Cmd_s shoot_cmd_send ; //发射控制指令的结构体
//shoot回传数据暂时不需要，如果后面要做摩擦轮温度闭环控制弹速的话，可在后面加上
//Shoot_Upload_Date_s

// robot整体工作状态
Robot_Status_e robot_state;          


void RobotCmdInit(void)
{
    fs16data = FSI6RemoteControlInit(&huart3);
    
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrol_Cmd_s)); //gimbal cmd control publisher register
    gimbal_cmd_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Date_s)); //gimbal feedback date subscriber register
    //失能的时候设置使能实的初始角度	
	  gimbal_cmd_send.pitch = 0.0f ;
    gimbal_cmd_send.yaw = 0.0f ; 
    
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrol_Cmd_s)); //shoot cmd control publisher register

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
        .can_handle = &hcan1 , //双机通信用can1 ，云台与底盘的其他电机用can2
        .rx_id = 0x312 ,   //对应底盘tx_id
        .tx_id = 0x311 ,   //对应底盘rx_id
        },
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s) ,  //send chassis cmd
        .recv_data_len = sizeof(Chassis_Upload_Data_s) , //recive chassis feedback date
    };
    cmd_can_comm = CANCommInit(&comm_conf) ; //先初始化，在把初始化的参数赋给实例

    robot_state = ROBOT_READY; // robot进入准备状态
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle ()
{
  float angle = gimbal_feedback.yaw_motor_single_round_angle ;
  if(angle > YAW_ALING_ANGLE && angle <= 180.f + YAW_ALING_ANGLE)
  {
   chassis_cmd_send.offset_angle = angle - YAW_ALING_ANGLE ;
  }
  else if (angle > 180.f + YAW_ALING_ANGLE )
  {
   chassis_cmd_send.offset_angle = angle - YAW_ALING_ANGLE - 360.0f ;
  }
  else 
  {
   chassis_cmd_send.offset_angle = angle - YAW_ALING_ANGLE ;  
  }
}


/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RoBotCmdRemoteControlSet(void)
{
    //FSI6 MODE CHOOSE/switch  
    //SC_LOGIC           ------->> gimbal and chassis mode
    if (fs16data->SC_CH7 == RC_SW_UP)
    {   
         // disable all robot
        chassis_cmd_send.chassis_mode = chassis_stop ;
        gimbal_cmd_send.gimbal_mode = GIMBAL_STOP ;   
    }
    else if (fs16data->SC_CH7 == RC_SW_MID)
    {   //enable all robot (follow mode)
        chassis_cmd_send.chassis_mode = chassis_follow ;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE ; 
    }
    else if (fs16data->SC_CH7 == RC_SW_DOWN && fs16data->V1_L > 200)
    {   
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE ;             
        chassis_cmd_send.chassis_mode = chassis_ZiZhua;                                          
        chassis_cmd_send.WZ = (fs16data->V2_R)-RC_KNOB_MIN; // 由V2映射小陀螺的速度,并建立死区防止误触(开启小陀螺，给WZ赋值)
    }

    //SB_AND_V2_LOGIC      ------>> shoot mode
    if(fs16data->V2_R>200)
    { //enable friction motor    
      shoot_cmd_send.shoot_mode = SHOOT_ON ;
      //choose loader motor mode
      switch (fs16data->SB_CH6)
      {
      case RC_SW_UP :
      shoot_cmd_send.loader_mode = LOAD_STOP ;       // no fire
        break;
      case RC_SW_DOWN :                               
      shoot_cmd_send.loader_mode = LOAD_BURSTFIRE ;  // fire
      default:
        break;
      }
    }

   //chassis建立死区
   fs16data->L_LR = abs(fs16data->L_LR) < 50.0f ? 0.0f : fs16data->L_LR ;   //绝对值是否小于50 ，是取0 ，否取通道值本身
   fs16data->L_UD = abs(fs16data->L_UD) < 50.0f ? 0.0f : fs16data->L_UD ; 
   //传入遥控参量
   chassis_cmd_send.VX = STICK_TO_SPEED_CHASSIS * (fs16data->L_UD);   // 前后平移
   chassis_cmd_send.VY = STICK_TO_SPEED_CHASSIS * (fs16data->L_LR);   // 左右平移
   //gimbal建立死区
   //pitch
   if( fs16data->R_UD > 50  )
   {
     gimbal_cmd_send.pitch += 0.5f ;     
   }
   else if ( fs16data->R_UD < -50)
   {
     gimbal_cmd_send.pitch -= 0.5f ;
   }
   //yaw
   if( fs16data->R_LR > 50 )
   {
     gimbal_cmd_send.yaw += 0.5f ;
   }
   else if ( fs16data->R_LR < -50)
   {
     gimbal_cmd_send.yaw -= 0.5f ;
   }
}

/* ROBOT核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RoBotCmdTask(void)
{   //双机通信传回数据（chassis---->gimbal） 
    chassis_feedback_date = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm); 
    //gimbal feedback date
    SubGetMessage(gimbal_cmd_sub,&gimbal_feedback);
    //shoot 暂时不加

    //计算对齐时的需要的WZ映射的速度
     CalcOffsetAngle();
    // ROBOTcontrolSet
    RoBotCmdRemoteControlSet();
    // send chassis_cmd
    CANCommSend(cmd_can_comm,(void *)&chassis_cmd_send);
    //send gimbal and shoot cmd
    PubPushMessage(gimbal_cmd_pub,(void *)&gimbal_cmd_send);
    PubPushMessage(shoot_cmd_pub ,(void *)&shoot_cmd_send) ;
}
