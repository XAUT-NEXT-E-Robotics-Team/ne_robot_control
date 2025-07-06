/**
 * @file remote_control.h
 * @author imgzw
 * @brief  遥控器模块定义头文件
 * @version 0.1
 * @date 2025-07-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef _REMOTE_CONTROL_H
#define _REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"

//FSI6遥控的数据
typedef struct 
{ 
    uint8_t FSI6Start;
    int16_t R_CH1;     //前后
    int16_t R_CH2;     //左右
    int16_t L_CH3;     //前后
    int16_t L_CH4;     //左右
    int16_t SA_CH5;
    int16_t SB_CH6;
    int16_t SC_CH7;
    int16_t SD_CH8;
    int16_t V1_CH9;
    int16_t V2_CH10;
    uint8_t FSI6_Flag;
    uint8_t FSI6_End;
}FSI6Data_t;



/*********外部接口**********/

/**
 * @brief FSI6遥控器模块初始化函数
 * 
 * @param rc_usart_handle 使用的串口号
 * @return RC_ctrl_t* 
 */
FSI6Data_t *FSI6RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);


#endif 
