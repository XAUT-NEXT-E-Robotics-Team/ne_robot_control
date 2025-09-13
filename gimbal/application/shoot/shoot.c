/**
 * @file shoot.c
 * @author cherishQA
 * @brief 
 * @version 0.1
 * @date 2025-07-12
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "cmsis_os.h"

//define frileftmotor frirightmotor  loadermotor 
DJIMotorInstance *frileftmotor ,*frirightmotor ,*loadermotor ;
//define shoot subscriber for geting cmd date
Subscriber_t  *shoot_sub ;
//define shoot publisher for sending feedback date
Publisher_t   *shoot_pub ;
//shoot cmd recive struct for 传参
Shoot_Ctrol_Cmd_s shoot_cmd_recv ; 
//shoot logic handle
Shoot_logic_handle_s shoot_logic_handle ;



void ShootInit()
{
  //frileftmotor frurightmotor  
  Motor_Init_Config_s  frictionmotor_config = {
   .can_init_config = {
    .can_handle = &hcan2 ,
   } ,
   .controller_param_init_config = {
    .speed_PID = {
        .Kp = 3.0f ,
        .Ki = 0.13f ,
        .Kd = 0.002f ,
        .IntegralLimit = 1000.0f ,
        .MaxOut = 16000.0f ,
        .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement),
    },
   },
   .controller_setting_init_config = {
    .speed_feedback_source = MOTOR_FEED , //feedback date from motor
		.outer_loop_type = SPEED_LOOP , 
    .close_loop_type = SPEED_LOOP , //choose cloop type
    .motor_reverse_flag = MOTOR_DIRECTION_NORMAL ,  //motor_run_deriction flag
   },
   .motor_type = M3508 , //choose motor type
  };
 //seting friction_motor
frictionmotor_config.can_init_config.tx_id = 1 ;
frileftmotor = DJIMotorInit(&frictionmotor_config);

frictionmotor_config.can_init_config.tx_id = 2 ;
frictionmotor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE ;  //反转
frirightmotor = DJIMotorInit(&frictionmotor_config);  
  
  //loadermotor
  Motor_Init_Config_s loadermotor_config ={
    .can_init_config = {
        .can_handle = &hcan1 ,
        .tx_id = 3 ,
    },                                //seting can inital
    .controller_param_init_config = {
        .speed_PID = {
            .Kp = 10.0f ,
            .Ki = 0.000f ,
            .Kd = 0.0f ,
            .IntegralLimit = 1000.0f ,
            .MaxOut = 13000.0f ,
            .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement),
        },
        .angle_PID = {
            .Kp = 0.3f ,
            .Ki = 0.00f ,
            .Kd = 0.0f ,
            .IntegralLimit = 2000.0f ,
            .MaxOut = 15000.0f ,
            .Improve = (PID_Improvement_e)(PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement),
        },      
    },
    .controller_setting_init_config = { //loadermotor use angle loop 
        .speed_feedback_source = MOTOR_FEED ,
        .angle_feedback_source = MOTOR_FEED ,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL ,
        .outer_loop_type = ANGLE_LOOP ,
        .close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP) } ,
        
    .motor_type = M3508 };              //seting motor type

    loadermotor = DJIMotorInit(&loadermotor_config);    //seting loadermotor

//finish publisher register and subscriber register
shoot_sub = SubRegister("shoot_cmd",sizeof(Shoot_Ctrol_Cmd_s));
//shoot_pub = PubRegister("shootfeedback",sizeof());   //shoot部分没有其他要传回来的了，可以直接在keil中查出对应的电机状态



  //初始化发射机构逻辑结构体
  shoot_logic_handle.loader_position = 0.0f ;
  shoot_logic_handle.shoot_count = 0 ;
  shoot_logic_handle.stuck_time_count = 0 ;

}


/****************************************TOOL函数***********************************************/
/**
 * @brief  loaader电机过零函数处理
 * @param  angle  目标角度 
 * @return 返回角度
*/
float Loader_zero_handle( float angle)
{
  if(angle >= 360.0f) angle -= 360.0f;
  if(angle <= 0.0f) angle += 360.0f;
  return angle ;
}

/**
 * @brief 4舍5入函数
 * @param num 
 * 
*/
uint16_t Round_half_up(float num)
{
  if( (num - (int)num) >= 0.5f ) return (int)num + 1 ;
  else return (int)num ;
}

float absf( float num )
{
 if( num < 0.0f ) return -num ;
 else  return num ;
}

/**
 * @brief 检查位置是否转到位
 * 
*/
uint8_t Loader_check_stuck( DJIMotorInstance* motor , float loader_position )
{
 static float err = 0.0f ;
  err =  absf( ( motor->measure.ecd * RALL_TO_ANGLE ) - loader_position ) ;
 if( err < 180.0f )  return err ;
 else return (360.0f - err) ;
}

/**
 * @brief 强制热量限制（以防计算错误）
 * 
*/
uint8_t Shoot_Heart_Limit( float max , float now )
{ 
  static float err = 0.0f ;
  err = max - now ;
  if( err <= SHOOT_HEAET_LIMIT )
  {
   return 0 ;
  }  
  else return 1 ;  
}
/****************************************TOOL函数***********************************************/

/**
 * @brief 发射执行函数
 * @param shoot 发射逻辑控制结构体
*/

void ShootRun( Shoot_logic_handle_s *shoot )
{
 switch( shoot->shooter_process )
 {
  case PROCESS_STOP :
    DJIMotorEnable(frileftmotor);
    DJIMotorEnable(frirightmotor);
    DJIMotorStop(loadermotor);
    DJIMotorSetRef(frileftmotor, 0.0f );
    DJIMotorSetRef(frirightmotor, 0.0f );   
   break;
  case PROCESS_HOLD :
    DJIMotorEnable(frileftmotor);
    DJIMotorEnable(frirightmotor);
    DJIMotorEnable(loadermotor);
    //seting shoot speed
    DJIMotorSetRef(frileftmotor,shoot->Bllute_Speed * SHOOT_Speed_uint);
    DJIMotorSetRef(frirightmotor,shoot->Bllute_Speed * SHOOT_Speed_uint);
    DJIMotorSetRef(loadermotor , shoot->loader_position ); //拨弹有力stop
    break;
  case PROCESS_FIR :
    DJIMotorEnable(frileftmotor);
    DJIMotorEnable(frirightmotor);
    DJIMotorEnable(loadermotor);

    DJIMotorSetRef(frileftmotor,shoot->Bllute_Speed * SHOOT_Speed_uint );
    DJIMotorSetRef(frirightmotor,shoot->Bllute_Speed * SHOOT_Speed_uint);
    DJIMotorSetRef(loadermotor , Loader_zero_handle( shoot->loader_position + LOADER_uint ) );      
    shoot->shoot_state = shoot_set ;
    break;
  case PROCESS_STUCK :
    DJIMotorEnable(frileftmotor);
    DJIMotorEnable(frirightmotor);
    DJIMotorEnable(loadermotor);

    DJIMotorSetRef(frileftmotor,shoot->Bllute_Speed * SHOOT_Speed_uint);
    DJIMotorSetRef(frirightmotor,shoot->Bllute_Speed * SHOOT_Speed_uint);
    DJIMotorSetRef(loadermotor , Loader_zero_handle( shoot->loader_position - LOADER_uint ) ); 
    shoot->shoot_state = shoot_set ;
    break;
  default :
  break;
 } 
} 
/**
 * @brief 卡弹处理
 * @param shoot 发射逻辑控制结构体
*/

void ShootManage( Shoot_logic_handle_s *shoot )
{
  if(  shoot->shoot_state == shoot_set && shoot->shooter_process == PROCESS_FIR )  //已开火
  {
    if( Loader_check_stuck( loadermotor , shoot->loader_position ) < LOADER_CHECK_Flag )
    { //在发发射模式下，判断位置没转到位                                            
      shoot->shoot_state = shoot_reset ;      //置位，已经发射
      shoot->loader_position = loadermotor->measure.ecd * RALL_TO_ANGLE ; //记录当前拨弹电机
      shoot->shooter_process = PROCESS_HOLD ;  //hold_mode
      shoot->stuck_time_count = 0 ;     //卡弹时间记0
    }
    else //没有转到位置   
      //判断电机的转速是否低于阈值
      if(  loadermotor->measure.speed_aps  
                         < LOADER_STUCK_SPEED ) {
         shoot->stuck_time_count ++ ;}

      //stuck_time_count时间超过0.5s的时候 ，认为已经卡弹了
    if( shoot->stuck_time_count > Round_half_up( SHOOT_STUCK_TINE_MAX ) )
      {
        shoot->stuck_time_count = 0 ; //计数清0
        shoot->shooter_process = PROCESS_STUCK ; //进入卡单处理
      }
  }
    
  //检查卡单处理是否成功 ，如果处理时间过长 ，强制认为处理好了  
  else if( shoot->shoot_state == shoot_set && shoot->shooter_process == PROCESS_STUCK  )
  {
    if( Loader_check_stuck( loadermotor, shoot->loader_position ) < LOADER_CHECK_Flag )
    {//如果处理好了
       shoot->shoot_state = shoot_reset ;  //重新置位    
       shoot->shooter_process = PROCESS_HOLD; //切换回HOLD模式
    }
    else { //如果没到位
       shoot->stuck_time_count ++ ;
       shoot->shooter_process = PROCESS_STUCK ; 
       if( shoot->stuck_time_count > Round_half_up( SHOOT_STUCK_TINE_MAX ) )
       {//强制认为处理完成
        shoot->stuck_time_count = 0 ;
        shoot->shooter_process = PROCESS_HOLD ;
        shoot->shoot_state =shoot_reset ;
       }
    }
  }
  
  ShootRun( shoot );
}
/**
 * @brief 热量限制
 * 
*/
void Shoot_control( Shoot_logic_handle_s * shoot , Shoot_Ctrol_Cmd_s  Cmd_recv  )
{
   //传入shooter需要的数据
   shoot->shoot_mode = Cmd_recv.shoot_mode_in ;
   shoot->Bllute_Speed = Cmd_recv.Blluet_speed_in ;
   shoot->shoot_heart_now = Cmd_recv.shoot_heart_now_in ;
   shoot->shoot_heart_max = Cmd_recv.shoot_heart_max_in ;

   //热量限制   Y = KX +B
   shoot->shoot_heart_freqlimit =
   -( SHOOT_PREQ_MAX / ( shoot->shoot_heart_max - shoot->shoot_heart_max/2 +  SHOOT_HEAET_LIMIT  ) )
   * ( shoot->shoot_heart_now - shoot->shoot_heart_max/2) + SHOOT_PREQ_MAX ;
   
   if( shoot->shoot_heart_freqlimit > SHOOT_PREQ_MAX ) 
   shoot->shoot_heart_freqlimit = SHOOT_PREQ_MAX;
   //发射计数
   shoot->shoot_count ++ ;
   if( shoot->shoot_heart_freqlimit < 1.0f ){
    shoot->shoot_count = 0 ;}
   else if( shoot->shoot_count > Round_half_up(SHOOT_WORK_PROQ/SHOOT_PREQ_MAX)) //时间间隔足够，计数不在延长，可以打弹
   { shoot->shoot_count = Round_half_up(SHOOT_WORK_PROQ/SHOOT_PREQ_MAX) ; }

  //模式选择
  switch( shoot->shoot_mode )
  {
   case SHOOTER_STOP :
    shoot->shooter_process = PROCESS_STOP ;
    break;
   case SHOOTER_HOLD :
    shoot->shooter_process = PROCESS_HOLD ;
    break;
   case SHOOTER_AUTO : 
    if( Shoot_Heart_Limit( shoot->shoot_heart_max , shoot->shoot_heart_now ) ){
      if( shoot->shooter_process == PROCESS_HOLD && shoot->shoot_count == Round_half_up(SHOOT_WORK_PROQ/SHOOT_PREQ_MAX) ) {
       //时间间隔到位，检查上一次是否为HOLD状态（上一颗弹丸打完）
       shoot->shooter_process =PROCESS_FIR ;
       //计数请零
       shoot->shoot_count = 0; }}
    else shoot->shoot_mode = SHOOTER_WAITING ;
    break;
   case SHOOTER_SINGEL:
    if( Shoot_Heart_Limit( shoot->shoot_heart_max , shoot->shoot_heart_now ) ){
      if( shoot->shooter_process == PROCESS_HOLD && shoot->shoot_count == Round_half_up(SHOOT_WORK_PROQ/SHOOT_PREQ_MAX) ) {
       //时间间隔到位，检查上一次是否为HOLD状态（上一颗弹丸打完）
       shoot->shooter_process = PROCESS_FIR ;
       shoot->shoot_count = 0 ;
       shoot->shoot_mode = SHOOTER_WAITING ; } }
    break;
   case SHOOTER_WAITING :
    if(shoot->shooter_process == PROCESS_HOLD){  //当状态回到PROCESS_HOLD时，MODE改为shooterhold ，说明发射空闲了
       shoot->shoot_mode = SHOOTER_HOLD ;}
    break; 
  }
 
 ShootManage( shoot);
}

void ShootTask() 
{
    //get cmd date
   SubGetMessage(shoot_sub,&shoot_cmd_recv);
	  //外部控制参数传入
   Shoot_control( &shoot_logic_handle ,  shoot_cmd_recv );
}
