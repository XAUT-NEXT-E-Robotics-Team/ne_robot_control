#ifndef  heat_limit_H
#define  heat_limit_H
#include "main.h"
#include "referee.h"
typedef struct 
{
 uint16_t Qmax ; //热量上限
 uint16_t Qcd ;  //热量冷却速度
 uint16_t Qnow ; //当前热量
 uint16_t Qwarn ;//热量预警 
 uint16_t Qres ; //热量剩余
 uint16_t Qsatu ;//热量饱和
 uint16_t Qthrsh ;//热量限阈值   
 uint16_t nmax ; //最大射频
 uint16_t shoot_frequency_limit  ; //射频限制 
} heat_limit_t ;

extern heat_limit_t shoot_speed_limit ;

void Shoot_Heat_limit_task(void) ;

#endif // 
