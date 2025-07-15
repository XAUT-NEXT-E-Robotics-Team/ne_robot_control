#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H
/* 
*@ 此部分云台底盘都使用
*/
#include "stdint.h"

/********
  / 
 /       (15°)
/    
--------------
\       
 \      （-15°） 
  \
********/
//GIMBAL
#define PITCH_MAX    15.0f        //云台向上最大角度
#define PITCH_MIN    -15.0f       //云台向下最大角度 
#define YAW_CHASSIS_ANGLE_ECD   2435   //根据具体云台和底盘对齐时的YAW轴的电机的ECD
#define PITCH_HORIZON_ECD    3000
        
//SHOOT
#define ONEBULLUTANGLE     15.0f  //一颗弹丸所占的角度
#define LOADEMOTOR_JSB     (36*2.5)     //减速比（电机加拨盘设计[后者问问机械]）
#define NUM_PER_CIRCLE     10     //拨盘一圈弹丸数量

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

//底盘模式设置
typedef enum 
{
  chassis_stop=0,       //有力停止模式
  chassis_follow,       //地盘跟随模式
  chassis_unfollow,     //地不跟随模式
  chassis_ZiZhua,       //小陀螺模式
}chassis_mode_e;

// 云台模式设置
typedef enum 
{
  GIMBAL_STOP =0 ,
  GIMBAL_FREE_MODE ,   //gimbal  自由模式（底盘为NO_FOLLOW）YAW电机反馈为taltol angel       
  GIMBAL_GYRO_MODE ,   //gimbal  陀螺仪反馈模式，反馈值为陀螺仪pitch,yaw_taltol angel
}gimbal_mode_e;

// 发射模式设置
typedef enum
{
 SHOOT_OFF = 0 ,       
 SHOOT_ON ,
}shoot_mode_e;

// 摩擦轮
typedef enum 
{
 FRICTION_OFF = 0 ,   //摩擦轮关闭
 FRICTION_ON          //摩擦轮开启
}friction_mode_e  ;

// 发射状态
typedef enum 
{
LOAD_STOP = 0 , //STOP SHOOTING
LOAD_REVERSE  , //反转
LOAD_1_BULLET , //shoot once
LOAD_2_BULLET , //shoot 2 
LOAD_3_BULLET , //shoot 3
LOAD_BURSTFIRE, //shoting 
}loader_mode_e;

// shoot_speed(need to pack to master_process.h)
typedef enum
{
 BULLET_SPEED_NONE = 0 ,
 BULLET_SPEED1 = 15   ,
 BULLET_SPEED2 = 18 
}Blluet_Speed_e;

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
 shoot_mode_e  shoot_mode ;           //发射   mode choose
 loader_mode_e loader_mode ;          //发射   state
 friction_mode_e friction_mode ;      //摩擦轮 state
 Blluet_Speed_e  Blluet_speed ;       //shoot speed
 uint8_t shoot_freqlimit ;            //射频限制
 float shoot_rate ;                   //当前射频 
}Shoot_Ctrol_Cmd_s;

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
uint8_t  shoot_freqlimit  ;//根据热量剩余量，设置弹频限制
uint8_t  shoot_Speed;      //弹速
Enemy_color_e Enemy_color ;//敌人的颜色  1 为RED   2为BLUE
} Chassis_Upload_Data_s;

typedef struct 
{
  //atitude_t gimbal_imu_date ; 
  uint16_t yaw_motor_single_round_angle;
}Gimbal_Upload_Date_s;

#pragma pack() //恢复默认对齐方式

#endif // ROBOT_DEF_H
