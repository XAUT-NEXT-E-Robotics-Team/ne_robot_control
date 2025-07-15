/**
 * @file remote_control.c
 * @author imgzw
 * @brief 对于遥控器接收数据处理的封装
 * @version 0.1
 * @date 2025-07-06
 * @todo 其他遥控器？键鼠/图传控制？
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "string.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

//遥控器数据
FSI6Data_t fsi6Data;
#define RC_FRAME_NUM     25u

//串口&守护进程实例
 USARTInstance *rc_usart_instance;
 DaemonInstance *rc_daemon_instance;

/**
 * @brief FSI6遥控器数据解析函数
 * 
 * @param buf 串口缓冲区
 */
static void GetFSI6Data(uint8_t *buf)
{
    fsi6Data.FSI6Start = buf[0];
    fsi6Data.R_LR = (((uint16_t)buf[1] >> 0 | ((uint16_t)buf[2] << 8 )) & 0x07FF)-RC_STICK_MID;
    fsi6Data.R_UD = (((uint16_t)buf[2] >> 3 | ((uint16_t)buf[3] << 5 )) & 0x07FF)-RC_STICK_MID;
    fsi6Data.L_UD = (((uint16_t)buf[3] >> 6 | ((uint16_t)buf[4] << 2 ) | (uint16_t)buf[5] << 10 ) & 0x07FF) - RC_STICK_MID;
    fsi6Data.L_LR = (((uint16_t)buf[5] >> 1 | ((uint16_t)buf[6] << 7 )) & 0x07FF)-RC_STICK_MID;
    fsi6Data.SA_CH5 = ((uint16_t)buf[6] >> 4 | ((uint16_t)buf[7] << 4 )) & 0x07FF;
    fsi6Data.SB_CH6 = ((uint16_t)buf[7] >> 7 | ((uint16_t)buf[8] << 1 ) | (uint16_t)buf[9] << 9 ) & 0x07FF;
    fsi6Data.SC_CH7 = ((uint16_t)buf[9] >> 2 | ((uint16_t)buf[10] << 6 )) & 0x07FF;
    fsi6Data.SD_CH8 = ((uint16_t)buf[10] >> 5 | ((uint16_t)buf[11] << 3 )) & 0x07FF;
    fsi6Data.V1_L = ((uint16_t)buf[12] >> 0 | ((uint16_t)buf[13] << 8 )) & 0x07FF;
    fsi6Data.V2_R = ((uint16_t)buf[13] >> 3 | ((uint16_t)buf[14] << 5 )) & 0x07FF;
    fsi6Data.FSI6_Flag = buf[23];
    fsi6Data.FSI6_End = buf[24]; 
}

/**
 * @brief 对GetFSI6Data的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    GetFSI6Data(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(&fsi6Data, 0, sizeof(fsi6Data)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

FSI6Data_t *FSI6RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = RC_FRAME_NUM;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };

    rc_daemon_instance = DaemonRegister(&daemon_conf);

    LOGERROR("[rc] remote control initialized"); // 输出初始化信息
    return &fsi6Data;
}

