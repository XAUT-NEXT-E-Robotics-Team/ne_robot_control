#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H
/* 
*@ 此部分云台底盘都使用
*/
#include "stdint.h"  
#include "ins_task.h"

//GIMBAL
#define PITCH_MAX    15.0f        //云台向上最大角度
#define PITCH_MIN    -15.0f       //云台向下最大角度 
#define YAW_CHASSIS_ANGLE_ECD   4458  //根据具体云台和底盘对齐时的YAW轴的电机的ECD
#define PITCH_HORIZON_ECD       5591
//小陀螺上下坐标系夹角计算
#define YAW_ECD_Greater_THAN_4096  0  //如果yaw轴电机软零点的ecd大于4096 ，此值为1 ，否则为0
//调试模式
#define WZ_control_to_V1   0          //是否启用小陀螺模式是更改WA速度控制量，以便视觉调自瞄 
#define WZ_control_to_V1_K 0.5        //WZ_control_to_V1模式下的比例系数  

//SHOOT
#define FSI6_V_DEAD_limit  200     //V1和V2软件死区限制，预防FSI6旋钮跑飞
 //执行PART
#define LOADEMOTOR_JSB     180     //减速比（电机加拨盘设计[后者问问机械]）
#define NUM_PER_CIRCLE     10      //拨盘一圈弹丸数量
#define SHOOT_Speed_uint   1910.0f   //1m/s线速度对应的W角加速度  ( 1/0.06 ) * 360.0
#define LOADER_uint        36.0f    //拨一个弹丸需要转的角度(注意loader电机的减速比)
#define LOADER_CHECK_Flag  LOADER_uint/2    //(LOADER_uint/2)
 //卡弹处理PART
#define LOADER_STUCK_SPEED 360.0f    //电机速度1rpm    
#define SHOOT_WORK_PROQ    200      //电机200HZ 
#define SHOOT_STUCK_TINE_MAX  (500/1000) * SHOOT_WORK_PROQ  //loader电机堵转时间判断卡弹  (500ms/1000ms)*1s运行的次数
 //热量限制PART
#define SHOOT_PREQ_MAX     18       //射频最大值     
#define SHOOT_HEAET_LIMIT  20.0f       //热量阈值，小于这个值直接进入等待 （计算 20 = 2 * 10即2发小弹丸）   
 //shoot              
#define SHOOT_Refeer_Enable  0

//CHASSIS
#define CENTER_GIMBAL_OFFSET_X 0.0f   //云台中心偏移x轴
#define CENTER_GIMBAL_OFFSET_Y 0.0f   //云台中心偏移y轴
#define RADIUS_WHEEL 60.0f            //轮子半径,单位mm   
#define REDUCTION_WHEEL 19.0f         //轮子减速比
#define R        0.3f               //半径  m

#pragma pack(1) 
//机器人状态
typedef enum 
{
  ROBOT_STOP=0,
  ROBOT_READY
} Robot_Status_e;

//APP状态
typedef enum 
{
  APP_OFFLINE=0,
  APP_ONLINE,
  APP_ERROR
}App_State_e;

//底盘模式设置                           ------------------chasiss part
typedef enum 
{
  CHASSIS_STOP=0,       //有力停止模式
  CHASSIS_FOLLOW,       //底盘跟随模式
  CHASSIS_UNFOLLOW,     //底盘不跟随模式
  CHASSIS_ZiZhua,       //小陀螺模式
}chassis_mode_e;

// 云台模式设置                           ------------------gimbal part
typedef enum 
{
  GIMBAL_STOP =0 ,
  GIMBAL_FREE_MODE ,   //gimbal  自由模式（底盘为NO_FOLLOW）YAW电机反馈为taltol angel  （ hero use）       
  GIMBAL_GYRO_MODE ,   //gimbal  陀螺仪反馈模式，反馈值为陀螺仪pitch,yaw_taltol angel
}gimbal_mode_e;


// 发射模式设置                           ------------------shoot part
typedef enum
{
 SHOOTER_STOP =0,  //失能所有电机
 SHOOTER_HOLD ,    //只开启摩擦轮     
 SHOOTER_AUTO ,    //自动     
 SHOOTER_SINGEL ,  //单发
 SHOOTER_WAITING   //等待
}shoot_mode_e;
//发射执行判断
typedef enum 
{
 PROCESS_STOP =0,  //失能所有电机
 PROCESS_HOLD ,    //开摩擦轮
 PROCESS_FIR ,     //发射处理
 PROCESS_STUCK ,   //卡弹处理
} shooter_process_e ;

//卡弹处理状态
typedef enum 
{
 shoot_set = 0  ,  //可以发射   
 shoot_reset       
}shoot_state_e  ;

// shoot_speed(need to pack to master_process.h)
typedef enum
{  //    m/s
 BULLET_SPEED_NONE = 0 ,
 BULLET_SPEED1 = 20   ,
 BULLET_SPEED2 = 24 
}Blluet_Speed_e;


//视觉部分                               ------------vision part
// enemy color
typedef enum 
{//Vsion_send
  enemecolor_none=0,
  enemycolor_red  ,
  enemycolor_blue
} Enemy_color_e ;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅

typedef struct 
{//CMD_CHASSIS_CONTROL
 float VX;          //底盘前进速度,单位mm/s
 float VY;          //底盘横向速度,单位mm/s
 float WZ;          //底盘角速度,单位rad/s
 float offset_angle; //底盘与归中位置的夹角
 chassis_mode_e  chassis_mode ;//底盘模式
 int chassis_speed_buff;
}Chassis_Ctrl_Cmd_s;

typedef struct 
{//CMD_GIMBAL_CONTROL
 float yaw    ;
 float pitch  ;
 float chassis_wz ;
 gimbal_mode_e gimbal_mode;
}Gimbal_Ctrol_Cmd_s;


typedef struct 
{//CMD_SHOOT_CONTROL

 shoot_mode_e  shoot_mode_in ;            //发射   mode choose

 shooter_process_e shooter_process_in ;   //发射处理进程   
 
 Blluet_Speed_e Blluet_speed_in ;         //射速设定
 
 //枪口热量处理
 float shoot_heart_now_in  ;              //当前热量
 
 float shoot_heart_max_in  ;              //热量最大值（裁判系统）
       
}Shoot_Ctrol_Cmd_s;

//发射逻辑处理结构体
typedef struct {

 shoot_mode_e shoot_mode ;            //发射   mode choose

 shooter_process_e shooter_process ;  //发射处理进程  

 shoot_state_e shoot_state ;          //发射状态

 Blluet_Speed_e   Bllute_Speed ;      //弹速

 //卡弹处理时间计数     
 uint16_t stuck_time_count ;
 
 //射频计数(保证实际频率与计划频率一致)
 uint16_t  shoot_count ;

 //拨弹实际角度记录
 float loader_position ;

 //枪口热量
 float shoot_heart_now  ;              //当前热量

 float shoot_heart_max  ;              //热量最大值（裁判系统）

 float shoot_heart_freqlimit ;       //射频限制

}Shoot_logic_handle_s;


/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */
//application subscriber struct

typedef struct 
{
//float chassis_vx;          //底盘前进速度,单位mm/s
//float chassis_vy;          //底盘横向速度,单位mm/s
//float chassis_wz;          //底盘角速度,单位rad/s
//shoot
uint16_t referee_shoot_heart_now ; //裁判系统传入的当前热量
uint16_t referee_shoot_heart_max ; //裁判系统传入的最大热量
//vision
Enemy_color_e Enemy_color ;//敌人的颜色  1 为RED   2为BLUE
} Chassis_Upload_Data_s;

typedef struct 
{
  attitude_t gimbal_imu_date ;
  uint16_t yaw_motor_single_round_angle;
}Gimbal_Upload_Date_s;

#pragma pack() //恢复默认对齐方式

#endif // ROBOT_DEF_H
