/**
 * @file 斜坡规划
 * @author cherishQA
 * @date 2025.7.28
 * @version 0.1 
 */

#include "Slope_Plan.h"
#include "arm_math.h"
#include "SuperPower_control.h"
Slope_Plan_e  Slope_Plan ;
SPEED_STATE_e SPEED_STATE ;

/**
 * @brief 斜坡规划器初始化函数
 * @param slope_plan 斜坡规划的结构体
 * @param target_x VX目标值
 * @param target_y VY目标值
 * @param motor1speed 电机1的速度
 * @param motor2speed 电机2的速度
 * @param motor3speed 电机3的速度
 * @param motor4speed 电机4的速度
 */

void Slope_Plan_init(Slope_Plan_e* slope_plan , float target_x ,float target_y, float motor1speed ,float motor2speed ,float motor3speed ,float motor4speed )
{
  slope_plan->K_add = 0.01f ;
  slope_plan->target_speed_VX =  target_x ;
  slope_plan->target_speed_VY =  target_y ;
  //运动学反解算
  if( (motor1speed - motor3speed) >0 && (motor2speed - motor4speed) > 0  )
  {
  slope_plan->now_speed_VX = (motor1speed - motor3speed) * 0.70710678f  + (motor2speed - motor4speed) * 0.70710678f ;
  slope_plan->now_speed_VY = -(motor1speed - motor3speed) * 0.70710678f + (motor2speed - motor4speed) * 0.70710678f ;
  }
  else if ( (motor1speed - motor3speed) >0 && (motor2speed - motor4speed)<0 )
  {
  slope_plan->now_speed_VX = (motor1speed - motor3speed) * 0.70710678f  +  - (motor2speed - motor4speed) * 0.70710678f ;
  slope_plan->now_speed_VY = -(motor1speed - motor3speed) * 0.70710678f +  - (motor2speed - motor4speed) * 0.70710678f ;   
  }
  else if ( (motor1speed - motor3speed )<0 && (motor2speed -motor4speed)>0 )
  {
  slope_plan->now_speed_VX = -(motor1speed - motor3speed) * 0.70710678f  + (motor2speed - motor4speed) * 0.70710678f ;
  slope_plan->now_speed_VY = (motor1speed - motor3speed) * 0.70710678f + (motor2speed - motor4speed) * 0.70710678f ;   
  }
  else if ( (motor1speed - motor3speed )<0 && ( motor2speed -motor4speed )<0)
  {
  slope_plan->now_speed_VX = -(motor1speed - motor3speed) * 0.70710678f  + -(motor2speed - motor4speed) * 0.70710678f ;
  slope_plan->now_speed_VY = (motor1speed - motor3speed) * 0.70710678f   + -(motor2speed - motor4speed) * 0.70710678f ;      
  }
  //规划值
  slope_plan->plan_speed_VX = 0.0f ;
  slope_plan->plan_speed_VY = 0.0f ;   
}


/**
 * @brief 斜坡规划器
 * @param Slope_Plan_e 斜坡规划的结构体
 * @return 返回规划值
 */

void Slope_Plan_work (Slope_Plan_e* slope_plan )
{ 
  //上一次的目标值  
  static float  last_target_x = 1.0f , last_target_y = 1.0f ; 
  
  if( (last_target_x *  slope_plan->plan_speed_VX) > 0 && ( fabs(slope_plan->plan_speed_VX) > fabs(last_target_x) ) )
  { //加速(上一次目标值与下一次目标值速度乘积大于0，且绝对值增加
    if( slope_plan->now_speed_VX < (slope_plan->target_speed_VX / 2 ) )
     {
        if( slope_plan->plan_speed_VX > 0)
        {   
        slope_plan->plan_speed_VX += ADD*slope_plan->K_add  ; 
        }
        else
        { 
        slope_plan->plan_speed_VX -= ADD*slope_plan->K_add ; 
        } 
     }
    else  
     {
       if( slope_plan->plan_speed_VX > (slope_plan->now_speed_VX + 0.05f) && slope_plan->plan_speed_VX < slope_plan->target_speed_VX  )
          slope_plan->plan_speed_VX = slope_plan->now_speed_VX  ;
     }
  }

  else if ( ((last_target_x *  slope_plan->plan_speed_VX) < 0)  ||    (((last_target_x *  slope_plan->plan_speed_VX) > 0) &&  (fabs(slope_plan->plan_speed_VX) < fabs(last_target_x)))  )
  {  //减速（上一次目标值与下一次目标值乘积小于0 或者 上一次目标值与下一次目标值速度乘积大于0且绝对值减少）
    if( slope_plan->now_speed_VX > (slope_plan->target_speed_VX / 2 ) )
     {
        if( slope_plan->plan_speed_VX < 0)
        {   
        slope_plan->plan_speed_VX += DECL*slope_plan->K_add  ; 
        }
        else
        { 
        slope_plan->plan_speed_VX -= DECL*slope_plan->K_add ; 
        }    

     }
    else 
     {
       if( slope_plan->plan_speed_VX < (slope_plan->now_speed_VX - 0.05f) && slope_plan->plan_speed_VX > slope_plan->target_speed_VX  )
          slope_plan->plan_speed_VX = slope_plan->now_speed_VX  ;
     }
  }


  
  if( (last_target_y *  slope_plan->plan_speed_VY) > 0 && ( fabs(slope_plan->plan_speed_VY) > fabs(last_target_y) ) )
  { //加速(上一次目标值与下一次目标值速度乘积大于0，且绝对值增加
    if( slope_plan->now_speed_VY < (slope_plan->target_speed_VY / 2 ) )
     {
        if( slope_plan->plan_speed_VY > 0)
        {   
        slope_plan->plan_speed_VY += ADD*slope_plan->K_add  ; 
        }
        else
        { 
        slope_plan->plan_speed_VY -= ADD*slope_plan->K_add ; 
        } 
     }
    else  
     {
       if( slope_plan->plan_speed_VY > (slope_plan->now_speed_VY + 0.05f) && slope_plan->plan_speed_VY < slope_plan->target_speed_VY  )
          slope_plan->plan_speed_VY = slope_plan->now_speed_VY  ;
     }
  }

  else if ( ((last_target_y *  slope_plan->plan_speed_VY) < 0)  ||    (((last_target_y *  slope_plan->plan_speed_VY) > 0) &&  (fabs(slope_plan->plan_speed_VY) < fabs(last_target_y)))  )
  {  //减速（上一次目标值与下一次目标值乘积小于0 或者 上一次目标值与下一次目标值速度乘积大于0且绝对值减少）
    if( slope_plan->now_speed_VY > (slope_plan->target_speed_VY / 2 ) )
     {
        if( slope_plan->plan_speed_VY < 0)
        {   
        slope_plan->plan_speed_VY += DECL*slope_plan->K_add  ; 
        }
        else
        { 
        slope_plan->plan_speed_VY -= DECL*slope_plan->K_add ; 
        }    

     }
    else 
     {
       if( slope_plan->plan_speed_VY < (slope_plan->now_speed_VY - 0.05f) && slope_plan->plan_speed_VY > slope_plan->target_speed_VY  )
          slope_plan->plan_speed_VY = slope_plan->now_speed_VY  ;
     }
  }
  //赋值上一次的参数
  last_target_x = slope_plan->target_speed_VX ;
  last_target_y = slope_plan->target_speed_VY ;
} 
