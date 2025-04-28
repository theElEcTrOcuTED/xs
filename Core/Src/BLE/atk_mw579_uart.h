/**
 ****************************************************************************************************
 * @file        atk_mw579_uart.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MW579ģ��UART�ӿ���������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� MiniSTM32 V4������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#ifndef __ATK_MW579_UART_H
#define __ATK_MW579_UART_H

#include "stm32f1xx.h"

/* ���Ŷ��� */
//#define ATK_MW579_UART_TX_GPIO_PORT         GPIOC
//#define ATK_MW579_UART_TX_GPIO_PIN          GPIO_PIN_12
//#define ATK_MW579_UART_TX_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

//#define ATK_MW579_UART_RX_GPIO_PORT         GPIOD
//#define ATK_MW579_UART_RX_GPIO_PIN          GPIO_PIN_2
//#define ATK_MW579_UART_RX_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)

#define ATK_MW579_UART_INTERFACE            USART2
#define ATK_MW579_UART_IRQn                 USART2_IRQn
#define ATK_MW579_UART_IRQHandler           USART2_IRQHandler
#define ATK_MW579_UART_CLK_ENABLE()         do{ __HAL_RCC_USART5_CLK_ENABLE(); }while(0)

/* UART�շ������С */
#define ATK_MW579_UART_RX_BUF_SIZE          2048
#define ATK_MW579_UART_TX_BUF_SIZE          64

/* �������� */
void atk_mw579_uart_printf(char *fmt, ...);     /* ATK-MW579 UART printf */
void atk_mw579_uart_rx_restart(void);           /* ATK-MW579 UART���¿�ʼ�������� */
uint8_t *atk_mw579_uart_rx_get_frame(void);     /* ��ȡATK-MW579 UART���յ���һ֡���� */
uint16_t atk_mw579_uart_rx_get_frame_len(void); /* ��ȡATK-MW579 UART���յ���һ֡���ݵĳ��� */
void atk_mw579_uart_init(uint32_t baudrate);    /* ATK-MW579 UART��ʼ�� */

#endif
