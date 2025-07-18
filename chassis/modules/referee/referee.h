#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"
//#include "CRC8_CRC16.h"
#include "fifo.h"
#include "main.h"
#include "protocol.h"
// #include "refereeData_v1.4.h"
// #include "refereeData_v1.5.h"
// #include "refereeData_v1.6.h"
#include "refereeData_v1.7.h"
#include "stdio.h"
#include "string.h"
#include "tim.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

void RefereeInit(void);
void referee_unpack_fifo_data(void);


#endif
