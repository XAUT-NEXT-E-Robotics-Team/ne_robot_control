#ifndef POWER_LIMIT_H
#define POWER_LIMIT_H
#include "main.h"

#include "dji_motor.h"

#define ERROR_POWER_DISTRIBUTION_SET 2000.0f
#define PROP_POWER_DISTRIBUTION_SET  1500.0f

float powerlimit_LVP_HK_pro(DJIMotorInstance *motor , float power );

#endif 
