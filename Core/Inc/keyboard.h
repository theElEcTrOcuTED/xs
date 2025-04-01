//
// Created by benea on 25-4-1.
//
//轮询法扫描矩阵键盘输入

#ifndef KEYBOARD_H
#define KEYBOARD_H

#include "stm32f1xx.h"

#define ROW_COUNT 4    //总行数
#define COL_COUNT 4    //总列数

//定义连接到的引脚，至于是GPIOA还是GPIOB的设定，去.c文件中改
#define ROW_PIN1 GPIO_PIN_0
#define ROW_PIN2 GPIO_PIN_1
#define ROW_PIN3 GPIO_PIN_2
#define ROW_PIN4 GPIO_PIN_3
#define COL_PIN1 GPIO_PIN_4
#define COL_PIN2 GPIO_PIN_5
#define COL_PIN3 GPIO_PIN_6
#define COL_PIN4 GPIO_PIN_7




void  keyboard_init();
char keyboard_scan_single();
void keyboard_scan_multi(char* result);
#endif //KEYBOARD_H
