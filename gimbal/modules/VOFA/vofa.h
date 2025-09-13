#ifndef  vofa_H
#define  vofa_H
#include "main.h"
#include "bsp_usart.h"

// 使用联合体，避免对数据的重复搬运
typedef struct VofaDatas {
  float set;
  float feedback;
  float  TargetSpeed1;
  float  Speed1;
  uint8_t tail1;
  uint8_t tail2;
  uint8_t tail3;
  uint8_t tail4;
} VofaData;

union VofaDATA {
  VofaData VofaD;
  uint8_t ch[20];  // 4*4+4=20
};

void JustFloat(float  set, float  feedback, float  target, float  speed, UART_HandleTypeDef *huart) ;

#endif
