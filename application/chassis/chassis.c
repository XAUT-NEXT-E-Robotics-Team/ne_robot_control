/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "motor_def.h"
/*根据robot_def.h中的macro自动计算的参数*/
#define HALF_WHEEL_BASE   (WHEEL_BASE / 2.0f)     //半轴长
#define HALF_TRACK_WIDTH  (TRACK_WIDTH / 2.0f)   //半轮距
#define PERIMETER_WHEEL   (RADIUS_WHEEL * 2.0F * PI) //轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */

static Publisher_t *chassis_pub  ;         //地盘信息发布者
static Subscriber_t *chassis_sub  ;        //地盘信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; //底盘接收控制命令
static Chassis_Upload_Data_s chassis_feedback_date;       //底盘上传数据
static DJIMotorInstance  *MOTOR1,*MOTOR2,*MOTOR3,*MOTOR4; //四个电机实例
/*                                    __
 *                          x        |\  
 *                          |          \ (w旋转速度方向)
 *                MOTOR2    |   MOTOR1  \
 *                          |
 *               y——————————|————————————
 *                          |
 *                MOTOR3    |   MOTOR4
 */
static float ZiZhuan_t  ;  //小陀螺自转的时间标志
static float chassis_vx , chassis_vy , chassis_vw ;  //云台速度的投影，平移速度，前进速度，旋转速度
static float chassis_motor1_speed , chassis_motor2_speed , chassis_motor3_speed , chassis_motor4_speed; //解算到四个电机的速度

void ChassisInit()
{
  //4个地盘轮子的参数相同，只有ID不同和正反转标志不同
  Motor_Init_Config_s chassis_motor_config = { 
    .can_init_config.can_handle =  &hcan2 ,
    .controller_param_init_config = {
      .speed_PID = {  
      .Kd = 0.5f,
      .Ki = 0.1f,
      .Kp = 0.01f,
      .IntegralLimit = 3000,
      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
      .MaxOut = 12000,
      },
      .current_PID ={
        .Kp = 0.5f,
        .Ki = 0.1f,
        .Kd = 0.01f,
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 12000,
      },
   },
    .controller_setting_init_config = {
    .angle_feedback_source = MOTOR_FEED, //角度反馈来源为电机
    .speed_feedback_source = MOTOR_FEED, //速度反馈来源为电机
    .feedforward_flag = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD, //前馈标志
    },
    .motor_type = M3508, //电机类型
  };
chassis_motor_config.can_init_config.tx_id = 1 ; //电机1的发送ID
MOTOR1 = DJIMotorInit(&chassis_motor_config); //电机1实例化

chassis_motor_config.can_init_config.tx_id = 2 ; //电机2的发送ID
MOTOR2 = DJIMotorInit(&chassis_motor_config); //电机2实例化

chassis_motor_config.can_init_config.tx_id = 3 ; //电机3的发送ID
MOTOR3 = DJIMotorInit(&chassis_motor_config); //电机3实例化

chassis_motor_config.can_init_config.tx_id = 4 ; //电机4的发送ID
MOTOR4 = DJIMotorInit(&chassis_motor_config); //电机4实例化

chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

}

/**
 * 
 * @brief 计算每个轮毂电机的输出,正运动学解算
 */
static void MecanumCalculate()
{
   chassis_motor1_speed =  (chassis_vx-chassis_vw*R)*arm_cos_fp32(45) + (chassis_vy + chassis_vw*R)*arm_sin_fp32(45) ;  
   chassis_motor2_speed =  (chassis_vx-chassis_vw*R)*arm_cos_fp32(135) + (chassis_vy + chassis_vw*R)*arm_sin_fp32(135) ;     
   chassis_motor2_speed =  (chassis_vx-chassis_vw*R)*arm_cos_fp32(-135) + (chassis_vy + chassis_vw*R)*arm_sin_fp32(-135) ;  
   chassis_motor2_speed =  (chassis_vx-chassis_vw*R)*arm_cos_fp32(-45) + (chassis_vy + chassis_vw*R)*arm_sin_fp32(-45) ;  
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(MOTOR1, chassis_motor1_speed);
    DJIMotorSetRef(MOTOR2, chassis_motor2_speed);
    DJIMotorSetRef(MOTOR3, chassis_motor3_speed);
    DJIMotorSetRef(MOTOR4, chassis_motor4_speed);
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
 SubGetMessage( chassis_sub , &chassis_cmd_recv);//订阅者接受数据
 if(chassis_cmd_recv.chassis_mode == chassis_stop)
 {//STOP_MODE
  DJIMotorStop(MOTOR1);
  DJIMotorStop(MOTOR2);
  DJIMotorStop(MOTOR3);
  DJIMotorStop(MOTOR4);
 }
 else { //WORK_MODE
  DJIMotorEnable(MOTOR1);
  DJIMotorEnable(MOTOR2);
  DJIMotorEnable(MOTOR3);
  DJIMotorEnable(MOTOR4);
 }

switch(chassis_cmd_recv.chassis_mode){
  case chassis_unfollow:
  chassis_vw = 0; //角速度
  break;
  case chassis_follow:
  chassis_vw = chassis_cmd_recv.WZ; //角速度
  break;
  case chassis_ZiZhua:
  chassis_vw = chassis_cmd_recv.WZ; //角速度
  //ZiZhuan_t += DWT_GetTimeDiff() / 1000.0
  break; //小陀螺模式
  default:
  break; //其他情况
}
  chassis_vx = chassis_cmd_recv.VX; //前进速度
  chassis_vy = chassis_cmd_recv.VY; //横向速度
// 根据控制模式进行正运动学解算,计算底盘输出
MecanumCalculate();

// 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
LimitChassisOutput();
//推送信息
PubPushMessage(chassis_pub,(void *)&chassis_feedback_date);

}














