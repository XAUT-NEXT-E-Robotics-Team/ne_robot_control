#ifndef SuperPower_control_H
#include "main.h"
#include "refereeData_v1.7.h"
/** 
 * @param    k_torque        （0.0036f）  CAN通信发1对应的扭矩
 * @param    k_uinit_switch  （6.28/60）  单位转换 （转速到弧度制）
 * 
*/
#define k_torque  0.0036f
#define k_uinit_switch  0.104719753334f        //(6.28/60)
//电机原始数据计算出来的功率  P_origin = f(w_now,i_origin) = ( w_now * (i_origin * 0.0036f) ) / (6.28/60)    


//电机功率建模
//计算电机功率所用的K0~K5参数(参数理论上应该用功率采样板，对电机[同一批电机参数应该是一样的]进行数据采集 ，然后用最小二乘法解出待定的系数K0~K5)
#define K0  0.577098513642893f
#define K1  0.84305930072969628f
#define K2  0.0027755999369711956f
#define K3  0.01763683292538948f
#define K4  0.15492097167594218f
#define K5  1.7697891966311674e-05f
//motor_power = K0 + K1 * I + K2 * W + K3 * I * W(机械功率) + K4 * I^2（铜损电热） + K5 * W^2（铁损电热）         (Maclaurin展开式)

//超电
#define SUPERCAP_RX_HEAD 0xF0F0
#define SUPERCAP_TX_HEAD 0xF1F2
// 超电接收结构体
typedef struct {
	uint16_t head;			// 包头 0xF0F0
    uint16_t raw_cap_power; // 原始值，单位0.01w
    float max_cap_power; // 电容最大提供功率，单位w
    uint16_t cap_percent;   // 电容剩余容量
    float input_power;   // 总输入功率
    uint16_t total_energy;  // 总消耗能量
    uint16_t err_code; // 0无错误
	uint8_t rx_buf[10];
}supercap_rx_t;


#pragma pack(1)

//超电发送结构体
typedef struct 
{
  uint16_t head ;               //包头0xF1F1
  uint8_t chassis_power_state ; //底盘供电状态
  uint16_t chassis_poweer_limit ; //底盘功率限制
} supercap_tx_t;

#pragma pack()

//超电部分
extern supercap_rx_t supercap_rxD ;
extern supercap_tx_t supercap_txD ;
extern uint8_t supercap_rx_flg ;
extern uint8_t supercap_err_flg ;
extern uint8_t referee_rx_flg ;
//功率限制部分
extern float P_origin[4]  ;           //4个电机的功率估计输出值
extern float P_chassis_total_origin ; //本次SPEED_PID输出的计算出来的输入的总功率 
extern float P_chassis_total_max  ;   //由裁判系统传入实际的值
extern float P_chassis_output ;       //功率限制最终的输出值 
extern float P_power  ;               //功率衰减系数


//超电
//超电接收解包
void Supercap_unpack( supercap_rx_t *supercap ) ;
//给超电发送数据
void Supercap_update_txd(supercap_tx_t* supercap_tx, robot_state_t* const robot_state) ;
//发送函数
void Supercap_transmit(UART_HandleTypeDef* huart ,supercap_tx_t * supercap_tx) ;


//功率限制
 float  CHassis_POWER_LIMIT ( float  chassis_power_total_origin );
//功率估算
 float MOTOR_POWER_ESTIMATE ( float motor_torque , float motor_w);
//电机实际发送电流计算
 float  MOTOR_TORQUE_FACT( float input_motorpower , float input_motor_origin_torque , float motor_w);

#endif // 
