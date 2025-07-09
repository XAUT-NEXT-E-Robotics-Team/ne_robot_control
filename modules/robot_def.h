#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "stdint.h"

#define CENTER_GIMBAL_OFFSET_X 0.0f   //云台中心偏移x轴
#define CENTER_GIMBAL_OFFSET_Y 0.0f   //云台中心偏移y轴
#define RADIUS_WHEEL 60.0f            //轮子半径,单位mm   
#define REDUCTION_WHEEL 19.0f         //轮子减速比
#define R        270.0f               //半径  mm

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

//地盘模式设置
typedef enum 
{
chassis_stop=0,       //有力停止模式
chassis_follow,       //地盘跟随模式
chassis_unfollow,     //地不跟随模式
chassis_ZiZhua,       //小陀螺模式
}chassis_mode_e;

typedef enum 
{
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

typedef struct {
//
float VX;          //底盘前进速度,单位mm/s
float VY;          //底盘横向速度,单位mm/s
float WZ;          //底盘角速度,单位rad/s
float offset_angle; //地盘与归中位置的夹角
chassis_mode_e  chassis_mode ;//地盘模式
int chassis_speed_buff;
}Chassis_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct {
//float chassis_vx;          //底盘前进速度,单位mm/s
//float chassis_vy;          //底盘横向速度,单位mm/s
//float chassis_wz;          //底盘角速度,单位rad/s
uint8_t  shoot_freqlimit  ;//根据热量剩余量，设置弹频限制
uint8_t  shoot_Speed;      //弹速
Enemy_color_e Enemy_color ;//敌人的颜色  1 为RED   2为BLUE
} Chassis_Upload_Data_s;


#pragma pack() //恢复默认对齐方式

#endif // ROBOT_DEF_H
