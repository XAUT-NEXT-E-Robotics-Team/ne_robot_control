/**
 * @file 热量限制
 * @author cherishQA
 * @date 2025.7.27
 * @version 0.1 
 */
#include "heat_limit.h"
#include "refereeData_v1.7.h"

heat_limit_t shoot_speed_limit ;

/**  
 *   /|\(射频 n/s)
 *    | 
 *    |_(Nmax)------------------------------------------------ ________________________________________________________________________
 *    |                                                      /|  
 *    |                                                     / |
 *    |                                                    /  | 
 *    |                                                   /   |
 *    |                                                  /    | 
 *    |                                                 /     |
 *    |                                                /      | 
 *    |_(Ncd)-----------______________________________/       | 
 *    |                 |                             |       |
 *    |                 |                             |       |  
 *    |                 |                             |       |
 *    |_________________|                             |       |(Qwarn)
 *    |------------------------------------------------------------------------------------------------------------------>(剩余热量 Qres)
 *                      |(Qthrsh)                     |(Qsatu)                     
 *@brief 热量限制
 */
 
void Shoot_Heat_limit_task(void)
{
 shoot_speed_limit.nmax = 18 ;      //最大弹频
 shoot_speed_limit.Qwarn = 200 ;    //热量预警
 shoot_speed_limit.Qsatu = 50 ;     //热量饱和
 shoot_speed_limit.Qthrsh = 20 ;    //热量限阈值 
 shoot_speed_limit.Qmax = robot_state.shooter_barrel_heat_limit ; //最大热量                                
 shoot_speed_limit.Qnow = power_heat_data.shooter_17mm_1_barrel_heat; //第一个17mm枪管的当前热量
 shoot_speed_limit.Qcd = robot_state.shooter_barrel_cooling_value ; //冷却速度
 shoot_speed_limit.Qres = shoot_speed_limit.Qmax - shoot_speed_limit.Qnow ; //剩余热量
 //射频限制 
 if(shoot_speed_limit.Qres >= shoot_speed_limit.Qwarn )
  {//最高射频
    shoot_speed_limit.shoot_frequency_limit = shoot_speed_limit.nmax ;
  }
 else { 
     if(shoot_speed_limit.Qres < shoot_speed_limit.Qwarn && shoot_speed_limit.Qres > shoot_speed_limit.Qsatu)
       {  //Qwarn~Qsatu
         shoot_speed_limit.shoot_frequency_limit =  ( (shoot_speed_limit.nmax - shoot_speed_limit.Qnow ) / 150 ) * (shoot_speed_limit.Qnow - shoot_speed_limit.Qsatu) + (shoot_speed_limit.Qcd / 10) ;}  
     else if ( shoot_speed_limit.Qres >= shoot_speed_limit.Qthrsh && shoot_speed_limit.Qres <= shoot_speed_limit.Qsatu )
       {  //Qsatu~Qthrsh
        shoot_speed_limit.shoot_frequency_limit = shoot_speed_limit.Qcd / 10 ;} 
     else if(shoot_speed_limit.Qres < shoot_speed_limit.Qthrsh )
       {  //0~Qthrsh
        shoot_speed_limit.shoot_frequency_limit = 0;}
    }
}
