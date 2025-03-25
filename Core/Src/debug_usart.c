//
// Created by benea on 25-3-24.
//

#include "debug_usart.h"

int CH_COUNT;
UART_HandleTypeDef *Huart;
int TIMEOUT;
void debug_usart_init(int channel_count,UART_HandleTypeDef *huart,int timeout) {
    CH_COUNT = channel_count;
    Huart = huart;
    TIMEOUT = timeout;
}

//一个浮点数据帧


void debug_usart_change_channel_number(int channel_number) {
    CH_COUNT = channel_number;
}
//justfloat格式传输

void debug_usart_send(float *data) {
    HAL_UART_Transmit(Huart,(uint8_t*)data,sizeof(float)*CH_COUNT,TIMEOUT);
    char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(Huart,(uint8_t*)tail,4,TIMEOUT);
}
