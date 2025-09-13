#ifndef Slop_Plan_H 
#define Slop_Plan_H 
#include "main.h"

#define  K_Reserve  0.70710678f  // 二分之根号二
#define   Rad_to_v  0.0314f         // (2 * 3.14 * R /60)    

typedef struct {
 float XYSpeed_now[2] ; //当前X，Y速度
 float XYSpeed_Plan[2] ;//规划X，Y速度
}slop_plan_e ;


void Kinematics_Reverse_Work( float  motor_speed_rpm[4] , slop_plan_e *slop_plan );
void Slop_Plan_work ( slop_plan_e *Slop ,float target_x ,float target_y ,float Add_Speed );
float absf ( float num ) ;

#endif //Slop_Plan_H     
