/**
 * @file chassis.c
 * @author cherishQA
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2025-7-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "usart.h"

#include "message_center.h"
#include "dji_motor.h"
#include "arm_math.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee.h"

#include "motor_def.h"
#include "can_comm.h"

#include "SuperPower_control.h"
#include "heat_limit.h"
#include "Slope_Plan.h"

extern UART_HandleTypeDef huart6; // 裁判系统USART句柄

/*根据robot_def.h中的macro自动计算的参数*/
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)        // 半轴长
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)      // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2.0F * PI) // 轮子周长

/*双机通信*/
CANCommInstance *chassis_can_comm ; 


/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */

// Publisher_t *chassis_pub;                            // 地盘信息发布者
// Subscriber_t *chassis_sub;                           // 地盘信息订阅者
Chassis_Ctrl_Cmd_s chassis_cmd_recv;                 // 底盘接收控制命令
Chassis_Upload_Data_s chassis_feedback_date;         // 底盘上传数据
DJIMotorInstance *MOTOR1, *MOTOR2, *MOTOR3, *MOTOR4; // 四个电机实例
/*                                    __
 *                          x        |\
 *                          |          \ (w旋转速度方向)
 *                MOTOR2    |   MOTOR1  \
 *                          |
 *               y——————————|————————————
 *                          |
 *                MOTOR3    |   MOTOR4
 */
// float ZiZhuan_t;                                                                              // 小陀螺自转的时间标志
float chassis_vx, chassis_vy, chassis_vw;                                                     // 云台速度的投影，平移速度，前进速度，旋转速度
float chassis_motor1_speed, chassis_motor2_speed, chassis_motor3_speed, chassis_motor4_speed; // 解算到四个电机的速度

void ChassisInit()
{
  // 4个地盘轮子的参数相同，只有ID不同和正反转标志不同
  Motor_Init_Config_s chassis_motor_config = {
      .can_init_config.can_handle = &hcan2,
      .controller_param_init_config = {
          .speed_PID = {
              .Kp = 5.1f, 
              .Kd = 0.005f,
              .Ki = 0.2f,
              .IntegralLimit = 0.0f,
              .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_DerivativeFilter ),
              .MaxOut = 16000.0f,
          },
      },
      .controller_setting_init_config = {
          .speed_feedback_source = MOTOR_FEED,
          .outer_loop_type = SPEED_LOOP,
          .close_loop_type = SPEED_LOOP,
      },
      .motor_type = M3508, // 电机类型
  };
  chassis_motor_config.can_init_config.tx_id = 3; // 电机1的发送ID
  MOTOR1 = DJIMotorInit(&chassis_motor_config);   // 电机1实例化

  chassis_motor_config.can_init_config.tx_id = 4; // 电机2的发送ID
  MOTOR2 = DJIMotorInit(&chassis_motor_config);   // 电机2实例化

  chassis_motor_config.can_init_config.tx_id = 1; // 电机3的发送ID
  MOTOR3 = DJIMotorInit(&chassis_motor_config);   // 电机3实例化

  chassis_motor_config.can_init_config.tx_id = 2; // 电机4的发送ID
  MOTOR4 = DJIMotorInit(&chassis_motor_config);   // 电机4实例化

  //双机通信CAN_comm初始化
  CANComm_Init_Config_s config = {
      .can_config = {
        .can_handle = &hcan1 ,
        .rx_id = 0x311,
        .tx_id = 0x312,
      },
      .send_data_len = sizeof(Chassis_Upload_Data_s) , //底盘上传数据
      .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),     //底盘接受云台CMD数据
  };
  chassis_can_comm = CANCommInit(&config);   // CHASSIS_BOARD
	
  RefereeInit(&huart6); // 初始化裁判系统,传入裁判系统的USART句柄
}

/**
 *
 * @brief 计算每个轮毂电机的输出,正运动学解算
 */
static void MecanumCalculate()
{
   chassis_motor1_speed =  ( (-0.707107f * chassis_vx) + (0.707107f  * chassis_vy) + chassis_vw*R) *1266.0f ;  
   chassis_motor2_speed =  ( (-0.707107f * chassis_vx) + (-0.707107f * chassis_vy) + chassis_vw*R) *1266.0f ;    
   chassis_motor3_speed =  ( (0.707107f  * chassis_vx) + (-0.707107f * chassis_vy) + chassis_vw*R) *1266.0f ;
   chassis_motor4_speed =  ( (0.707107f  * chassis_vx) + (0.707107f  * chassis_vy) + chassis_vw*R) *1266.0f ;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
 //功率限制 
 //4个电机的功率估计值
   P_origin[0] = MOTOR_POWER_ESTIMATE(chassis_motor1_speed , MOTOR1->measure.speed_aps);
   P_origin[1] = MOTOR_POWER_ESTIMATE(chassis_motor2_speed , MOTOR2->measure.speed_aps);
   P_origin[2] = MOTOR_POWER_ESTIMATE(chassis_motor3_speed , MOTOR3->measure.speed_aps);
   P_origin[3] = MOTOR_POWER_ESTIMATE(chassis_motor4_speed , MOTOR4->measure.speed_aps);
 //求4个电机的总功率
   P_chassis_total_origin  = P_origin[0] + P_origin[1] + P_origin[2] + P_origin[3] ;

 //更新底盘当前状态
   Supercap_update_txd(&supercap_txD, &robot_state);
 //发送底盘当前状态
   Supercap_transmit(&huart1 ,&supercap_txD);
 //接受超电的反馈值  
   Supercap_unpack(&supercap_rxD);
 //更新机器人的实际功率限制
   if(supercap_err_flg == 1) 
   {//如果超电出错
   P_chassis_total_max = robot_state.chassis_power_limit  ;}
   else { //等待加入图传链路控制(按键控制开超电)
    // if( ) { chassis_max_power = supercap_rxD.max_cap_power + robot_state.chassis_power_limit ; }
    // else {     
   P_chassis_total_max = robot_state.chassis_power_limit ;
   };

 //计算衰减比例
 if( P_chassis_total_origin > P_chassis_total_max){
   P_power =  CHassis_POWER_LIMIT (P_chassis_total_origin);
   //按比例分别衰减对应的电机功率
   P_origin[0] = P_power * P_origin[0] ;
   P_origin[1] = P_power * P_origin[1] ;
   P_origin[2] = P_power * P_origin[2] ;
   P_origin[3] = P_power * P_origin[3] ;
   //根据衰减之后的电机功率计算电机对应的扭矩电流
   chassis_motor1_speed =  MOTOR_TORQUE_FACT(P_origin[0],chassis_motor1_speed,MOTOR1->measure.speed_aps);
   chassis_motor2_speed =  MOTOR_TORQUE_FACT(P_origin[1],chassis_motor2_speed,MOTOR2->measure.speed_aps);
   chassis_motor3_speed =  MOTOR_TORQUE_FACT(P_origin[2],chassis_motor3_speed,MOTOR3->measure.speed_aps);
   chassis_motor4_speed =  MOTOR_TORQUE_FACT(P_origin[3],chassis_motor4_speed,MOTOR4->measure.speed_aps);

   }
  else{ } //如果没有超过功率限制，直接把PID算出来的扭矩电流赋值给电机


  // 完成功率限制后进行电机参考输入设定
  DJIMotorSetRef(MOTOR1, chassis_motor1_speed);
  DJIMotorSetRef(MOTOR2, chassis_motor2_speed);
  DJIMotorSetRef(MOTOR3, chassis_motor3_speed);
  DJIMotorSetRef(MOTOR4, chassis_motor4_speed);
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
  //上板命令传入下板
  chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s*)CANCommGet(chassis_can_comm);

  if (chassis_cmd_recv.chassis_mode == chassis_stop)
  { // STOP_MODE
    DJIMotorStop(MOTOR1);
    DJIMotorStop(MOTOR2);
    DJIMotorStop(MOTOR3);
    DJIMotorStop(MOTOR4);
  }
  else
  { // WORK_MODE
    DJIMotorEnable(MOTOR1);
    DJIMotorEnable(MOTOR2);
    DJIMotorEnable(MOTOR3);
    DJIMotorEnable(MOTOR4);
  }

  switch (chassis_cmd_recv.chassis_mode)
  {
  case chassis_unfollow:
    chassis_cmd_recv.WZ = 0; // 角速度
    break;
  case chassis_follow:
    chassis_vw = chassis_cmd_recv.WZ; // 角速度
    break;
  case chassis_ZiZhua:
    chassis_vw = 50; // 角速度
    break; // 小陀螺模式
  default:
    chassis_cmd_recv.WZ = 0; // 角速度
    break; // 其他情况
  }
  chassis_vx = chassis_cmd_recv.VX ; // 前进速度
  chassis_vy = chassis_cmd_recv.VY ; // 横向速度
  
  //斜坡规划
  Slope_Plan_init(&Slope_Plan,chassis_vx,chassis_vy, MOTOR1->measure.speed_aps , MOTOR2->measure.speed_aps  ,MOTOR3->measure.speed_aps ,MOTOR4->measure.speed_aps );
  Slope_Plan_work( &Slope_Plan );
  chassis_vx = Slope_Plan.plan_speed_VX ;
  chassis_vy = Slope_Plan.plan_speed_VY ;

  //热量限制
  Shoot_Heat_limit_task();
  //弹频
  chassis_feedback_date.shoot_freqlimit  =  shoot_speed_limit.shoot_frequency_limit ;

  // 根据控制模式进行正运动学解算,计算底盘输出
  MecanumCalculate();

  // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
  LimitChassisOutput();

  // 反馈数据给上板
  CANCommSend(chassis_can_comm,(void *)&chassis_feedback_date);
}
