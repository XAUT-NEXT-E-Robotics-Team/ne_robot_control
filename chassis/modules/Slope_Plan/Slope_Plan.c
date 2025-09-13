/**
 * @file 斜坡规划
 * @author cherishQA
 * @date 2025.7.28
 * @version 0.3
 */

#include "Slope_Plan.h"
#include "arm_math.h"

float absf ( float num )
{
 if( num > 0 ) return num ;
 else return (-num) ;
}
/**
 * @brief 运动学放反解算
 * @param motor_speed_rpm 4个motor的速度，单位是rpm
 * @param slop_plan  反解算X，Y速度
 * 
*/
void Kinematics_Reverse_Work( float  motor_speed_rpm[4] , slop_plan_e *slop_plan ){

   static float MOTOR13 ,MOTOR24 ,MOTOR_W ;
	 MOTOR_W = (motor_speed_rpm[0]+motor_speed_rpm[1]+motor_speed_rpm[2]+motor_speed_rpm[3])/4.0f;
   MOTOR13 = motor_speed_rpm[0] + motor_speed_rpm[2] - 2*MOTOR_W ;
   MOTOR24 = motor_speed_rpm[1] + motor_speed_rpm[3] - 2*MOTOR_W ;

   if( MOTOR13 > 0 ) 
   {
    slop_plan->XYSpeed_now[0] =  MOTOR13 * (K_Reserve) *Rad_to_v ;
    slop_plan->XYSpeed_now[1] =  MOTOR13 *(-K_Reserve) *Rad_to_v;
   }
   else 
   {
    slop_plan->XYSpeed_now[0] = MOTOR13 * (-K_Reserve) *Rad_to_v;
    slop_plan->XYSpeed_now[1] = MOTOR13 * (K_Reserve) *Rad_to_v;
   }

   if( MOTOR24 > 0) 
   {
    slop_plan->XYSpeed_now[0] +=  MOTOR24 * ( K_Reserve) *Rad_to_v;
    slop_plan->XYSpeed_now[1] +=  MOTOR24 * ( K_Reserve) *Rad_to_v;
   }
   else 
   {
    slop_plan->XYSpeed_now[0] +=  MOTOR24 * ( -K_Reserve) *Rad_to_v;
    slop_plan->XYSpeed_now[1] +=  MOTOR24 * ( -K_Reserve) *Rad_to_v;
   }
}

/**
 * @brief 斜坡规划器
 * @param Speed_target 参考速度
 * @param Add_Speed 每次增加或减小的速度(绝对值)
 * @param Speed_now 当前值
 * @return  规划值
*/
void Slop_Plan_work ( slop_plan_e *Slop ,float target_x ,float target_y ,float Add_Speed )
{
  //x
	if( target_x > 0.0f ){
		if( Slop->XYSpeed_now[0]  < target_x)
    {
      Slop->XYSpeed_Plan[0] += Add_Speed ;
  	}
	  else Slop->XYSpeed_Plan[0] -= Add_Speed ;
		
		if( Slop->XYSpeed_Plan[0] > target_x ) Slop->XYSpeed_Plan[0] = target_x ;
	 }
  else if( target_x < 0.0f ) {
	  if( target_x < Slop->XYSpeed_now[0] )
    {
      Slop->XYSpeed_Plan[0] -= Add_Speed ;
  	}
	  else Slop->XYSpeed_Plan[0] += Add_Speed ;
	  if( Slop->XYSpeed_Plan[0] < target_x ) Slop->XYSpeed_Plan[0] = target_x ;
	  }
	else if( target_x == 0.0f ){ 
	  Slop->XYSpeed_Plan[0] = 0.0f ; 
	}
	

  //y	
	if( target_y > 0.0f ){
		if( Slop->XYSpeed_now[1]  < target_y)
    {
      Slop->XYSpeed_Plan[1] += Add_Speed ;
  	}
	  else Slop->XYSpeed_Plan[1] -= Add_Speed ;
		
		if( Slop->XYSpeed_Plan[1] > target_y ) Slop->XYSpeed_Plan[1] = target_y ;
	 }
  else if( target_y < 0.0f ) {
	  if( target_y< Slop->XYSpeed_now[1] )
    {
      Slop->XYSpeed_Plan[1] -= Add_Speed ;
  	}
	  else Slop->XYSpeed_Plan[1] += Add_Speed ;
	  if( Slop->XYSpeed_Plan[1] < target_y ) Slop->XYSpeed_Plan[1] = target_y ;
	  }
	else if( target_y == 0.0f ){ 
	  Slop->XYSpeed_Plan[1] = 0.0f ; 
	}
}

