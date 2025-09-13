#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H
#include "main.h"

#include "dji_motor.h"

typedef struct  
{
  //电机功率模型 P = I * v *teccoff + k2 * v^2 + a * I^2 + C  
   float T_coff ; 
   float constant ; 
   float K2 ;
   float a ;

} Powerlimit_e ;  


extern Powerlimit_e Powerlimit ;

#define ERROR_POWER_DISTRIBUTION_SET 2000.0f
#define PROP_POWER_DISTRIBUTION_SET  1500.0f

float powerlimit_LVP_HK_pro(DJIMotorInstance *motor , float power );

#endif 
