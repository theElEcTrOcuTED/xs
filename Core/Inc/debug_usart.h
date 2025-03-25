//
// Created by benea on 25-3-24.
//


//串口调试
#ifndef DEBUG_USART_H
#define DEBUG_USART_H
#include "usart.h"


void debug_usart_init(int channel_count,UART_HandleTypeDef *huart,int timeout);
void debug_usart_change_channel_number(int channel_number);
void debug_usart_send(float *data);
#endif //DEBUG_USART_H
