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

// FSI6遥控的数据
typedef struct
{
    uint8_t FSI6Start;
    int16_t R_LR; // 前后
    int16_t R_UD; // 左右
    int16_t L_UD; // 前后
    int16_t L_LR; // 左右
    int16_t SA_CH5;
    int16_t SB_CH6;
    int16_t SC_CH7;
    int16_t SD_CH8;
    int16_t V1_L;
    int16_t V2_R;
    uint8_t FSI6_Flag;
    uint8_t FSI6_End;
} FSI6Data_t;

// 按键状态
typedef enum
{
    RC_SW_UP = 240,     // 按键最上
    RC_SW_MID = 1024,   // 按键中间值
    RC_SW_DOWN = 1807,  // 按键最下
} RC_SW_STATE_e;

typedef enum
{   
    RC_STICK_MAX = 1807,    // 遥控器摇杆最大值
    RC_STICK_MID = 1024,    // 遥控器摇杆中间值
    RC_STICK_MIN = 240,     // 遥控器摇杆最小值
}RC_STICK_STATE_e;

typedef enum
{
   RC_KNOB_MAX = 1807,  // 遥控器旋钮最大值
   RC_KNOB_MIN = 240,   // 遥控器旋钮最小值
}RC_KNOB_STATE_e;

/*********外部接口**********/

/**
 * @brief FSI6遥控器模块初始化函数
 *
 * @param rc_usart_handle 使用的串口号
 * @return RC_ctrl_t*
 */
FSI6Data_t *FSI6RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

#endif
