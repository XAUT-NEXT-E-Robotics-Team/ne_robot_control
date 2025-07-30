#ifndef Slop_Plan_H 
#define Slop_Plan_H 
#include "main.h"

typedef struct {
 float  target_speed_VX ;  //VX目标值
 float  target_speed_VY ;  //VY目标值
 float  now_speed_VX ;       //VX当前值或者叫真实值
 float  now_speed_VY ;       //VY当前值或者叫真实值
 float  plan_speed_VX ; //VX规划值
 float  plan_speed_VY ; //VY规划值
 float  K_add ;         //每次计算周期加的值 （根据你的最大速度和计算频率决定）
} Slope_Plan_e ;

typedef enum 
{
 ADD = 1 ,
 DECL = -1  ,
} SPEED_STATE_e;

extern Slope_Plan_e  Slope_Plan ;
extern SPEED_STATE_e SPEED_STATE ;

void Slope_Plan_init(Slope_Plan_e* slope_plan , float target_x ,float target_y, float motor1speed ,float motor2speed ,float motor3speed ,float motor4speed );
void Slope_Plan_work (Slope_Plan_e* slope_plan );

#endif //Slop_Plan_H     
