/**
 * @author cherishQA
 * @file   功率限制
 * @version  0.1
 * @date   2025.7.22
 */

#include "SuperPower_control.h"
//裁判系统

#include "referee.h"
#include "arm_math.h"
#include "usart.h"


float P_origin[4]  ;        //4个电机的功率估计输出值
float P_chassis_total_origin =0.0f ; //本次SPEED_PID输出的计算出来的输入的总功率 
float P_chassis_total_max = 50.0  ;  //由裁判系统传入实际的值
float P_chassis_output = 0.0f ;      //功率限制最终的输出值 
float P_power =1.0f ;                //功率衰减系数
//超电
supercap_rx_t supercap_rxD ;
supercap_tx_t supercap_txD ;

uint8_t supercap_rx_flg = 0 ;
uint8_t supercap_err_flg = 0 ;
uint8_t referee_rx_flg = 0 ;

/**
 * @file 超电接收解包
 * @param supercap   超电接收结构体
 */

void Supercap_unpack( supercap_rx_t *supercap ) 
{
  if(supercap->err_code != 0) 
  {
   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, supercap_rxD.rx_buf ,sizeof(supercap_rxD.rx_buf)); 
  }
  supercap->head = supercap->rx_buf[0] << 8 | supercap->rx_buf[1] ;
  if(supercap->head == SUPERCAP_RX_HEAD && supercap_rx_flg == 1)
  {
   supercap->raw_cap_power = (supercap->rx_buf[2] | supercap->rx_buf[3] << 8);           //原始值 0.01w
   supercap->max_cap_power = (supercap->rx_buf[2] | supercap->rx_buf[3] << 8) * 0.01f ;  //电容最大提供功率 w
   supercap->cap_percent = (supercap->rx_buf[4] | supercap->rx_buf[5] << 8 );            //电容余量   
   supercap->input_power = (supercap->rx_buf[6] | supercap->rx_buf[7] << 8 );            //总输入功率
   supercap->err_code = (supercap->rx_buf[8] | supercap->rx_buf[9] << 8 );                
   
   if(supercap->max_cap_power >= 50) // 保证超电提供的功率不超过50W
   {
    supercap_rxD.max_cap_power = 50 ;
   }
   else if (supercap->max_cap_power < 0 )
   {
    supercap_rxD.max_cap_power = 0 ;
   }
  
  if(supercap->err_code != 0 ){ 
     supercap_err_flg = 1 ; 
  }
  else { supercap_err_flg = 0 ;}
  }
}


/**
 * @file 给超电发送数据
 * @param robot_state 机器人状态
 * @param supercap_tx 超电发送结构体
 */
void Supercap_update_txd(supercap_tx_t* supercap_tx, robot_state_t* const robot_state) 
{
  supercap_tx->head = SUPERCAP_TX_HEAD ;
  if(referee_rx_flg == 1)
  {
    supercap_tx->chassis_poweer_limit = robot_state->chassis_power_limit ;
    supercap_tx->chassis_power_state  =robot_state->power_management_chassis_output ;
  }
  else 
  {
    supercap_tx->chassis_poweer_limit = 50 ;
    supercap_tx->chassis_power_state = 1 ;
  }
}

/**
 * @file 发送函数
 * @param huart 句柄
 * @param supercap_tx 发送结构体
 */
void Supercap_transmit(UART_HandleTypeDef* huart ,supercap_tx_t * supercap_tx)
{
 HAL_UART_Transmit_DMA( huart , (uint8_t*)supercap_tx , sizeof(supercap_tx_t)); 
}



/**
 * @file 浮点数绝对值函数
 */
float  abs_fp32 (float input)
{
  if(input>0)  return input ;
  else  return -input ;
} 

/**
 * 
 * @function  衰减功率方法,算出衰减的比例，映射到4个扭矩电流上
 * @param     chassis_power_total_origin   4个电机的输出功率估计值
 * @return    k_power
 */

 float  CHassis_POWER_LIMIT ( float  chassis_power_total_origin )
 {
    float k_p = 1.0f;
    if( chassis_power_total_origin <= P_chassis_total_max  ) {//判断是否大于总的功率
       return k_p ;}
    
    else {  //计算衰减比例
      k_p = P_chassis_total_max / chassis_power_total_origin ;  
      return k_p ; 
    }
 } 

/**
 * 
 * @param MOTOR_POWER_ESTIMATE  电机功率估计值
 * @param motor_torque          电机的扭矩电流的origin值
 * @param motor_w               电机的转速
 * 
 */
float MOTOR_POWER_ESTIMATE ( float motor_torque , float motor_w)
{
  float motor_torque_out = 0.0f ;
  motor_torque_out = (motor_w * (motor_torque * k_torque)) * k_uinit_switch ;
  return motor_torque_out ;
}

/**
 * @brief 根据衰减后的功率反解出电机的扭矩电流值
 * @param input_motorpower 输入的电机衰减后的功率
 * @param motor_w  电机的转速
 * @param input_motor_origin_torque 电机的origin电流
 * @return 返回电机用的电流值
 */
float  MOTOR_TORQUE_FACT( float input_motorpower , float input_motor_origin_torque , float motor_w)
{
  float output_motortorque1 = 0.0 ;
  float output_motortorque2 = 0.0 ;
  float root_e = 0.0f ;
  float root = 0.0f ;
  //b^2 - 4 * a * c  
  root_e = (K1 + K3 * motor_w) * (K1 + K3 * motor_w) - 4 * K4 * ( -input_motorpower + K0 + K2 * motor_w + K5 * motor_w * motor_w  ) ;

  if( root_e >0 ) {  //>0 , 有2解 
    
     arm_sqrt_f32( root_e , &root); 
     //求根公式
     output_motortorque1 = (-(K1 + K3 * motor_w )  +  root  ) / ( 2 * K4 ) ;
     output_motortorque2 = (-(K1 + K3 * motor_w )  -  root  ) / ( 2 * K4 ) ;
     if( abs_fp32( input_motor_origin_torque-output_motortorque1 ) < abs_fp32( input_motor_origin_torque-output_motortorque2 ) )
      { return output_motortorque1; }
     else { return output_motortorque2 ;}   
    
    }
  else if (root_e == 0 ) { //只有一个根
  
     output_motortorque1 = (-(K1 + K3 * motor_w )  +  root  ) / ( 2 * K4 ) ;  // output_motortorque2  = output_motortorque1 二者相等，随便返回那个值都行
     return output_motortorque1 ;
    }
  else if ( root_e<0 ) {  //无解（此时说明不在电机的正常值范围内，安全起见，直接赋值未0）

    return 0 ;
    }
   return 0 ; 
}


