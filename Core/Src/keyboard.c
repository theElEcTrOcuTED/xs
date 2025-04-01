//
// Created by benea on 25-4-1.
//

#include "keyboard.h"

#include <stdlib.h>
//同样是定义引脚
//写C的都没有对象，sb玩意没有命名空间真恶心
GPIO_TypeDef *KEYBOARD_ROW_PORT = GPIOA;
GPIO_TypeDef *KEYBOARD_COL_PORT = GPIOB;
//如果改了上面这两个，记得去.c文件中把初始化的RCC使能GPIO时钟也改了

//按钮映射图
const char KEYBOARD_BUTTON_MAP[ROW_COUNT][COL_COUNT] =
    {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};

void  keyboard_init() {
    //使能时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //行IO初始化
    //行设置为输出模式，推挽输出，不使能上下拉，用来输出检测信号
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = ROW_PIN1 | ROW_PIN2 | ROW_PIN3 | ROW_PIN4;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(KEYBOARD_ROW_PORT, &GPIO_InitStructure);
    //列IO初始化
    //列设置为输入模式，使能上拉电阻，这样当行输入低电平时，若某一列的按键被按下，这一列读取到的电平将由高变低

    GPIO_InitTypeDef GPIO_InitStructure2;
    GPIO_InitStructure2.Pin = COL_PIN1 | COL_PIN2 | COL_PIN3 | COL_PIN4;
    GPIO_InitStructure2.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure2.Pull = GPIO_PULLUP;//使能上拉电阻
    GPIO_InitStructure2.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(KEYBOARD_COL_PORT, &GPIO_InitStructure2);
}

/**
 * 扫描检测哪个按键被按下了。这个函数只能够检测一个按键。
 * 没有消抖功能，使用时请自行消抖（触发一次后延迟200ms再检测etc）
 * @return  按下的按键值，为char = 1~16 ，不是字符！！是0x01,0x02...，若没有按键按下，返回0x00
 */
char keyboard_scan_single() {
    //遍历每一行：
    for(int row = 0; row < ROW_COUNT; row++) {
        //输出部分
        for(int i = 0 ; i < ROW_COUNT; i++) {
            //对于当前行，设置输出为低电平，其他行全部输出高电平
            if(i == row) {
                HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, (1<<i), GPIO_PIN_RESET);
            }
            else {
                HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, (1<<i), GPIO_PIN_SET);
            }
        }
        //扫描部分：然后逐列检测IO引脚电平
        for(int col = 0 ; col < COL_COUNT; col++) {
            //本来使能了上拉电阻，如果读到低电平，说明对应按键被按下
            if(HAL_GPIO_ReadPin(KEYBOARD_COL_PORT, col) == GPIO_PIN_RESET) {
                //说明这一行被按下了
                return KEYBOARD_BUTTON_MAP[row][col];
            }
        }
    }
    return 0;
}

/**
 * 扫描检测哪些按键被按下了。没有消抖功能，使用时请自行消抖（触发一次后延迟200ms再检测etc）
 * @param result 传入一个行数为ROW_COUNT，列数为COL_COUNT的二维数组指针，
 * 该函数会修改该二维数组，若对应位置的按键被按下，则该位置为0x01，否则0x00（注意不是'1'，'0'的字符）
 * 请注意，二维数组的行数和列数必须与keyboard系统的总行数列数严格相等，否则计算索引值时会出现异常
 */
void keyboard_scan_multi(char* result) {
    //遍历每一行：
    for(int row = 0; row < ROW_COUNT; row++) {
        //输出部分
        for(int i = 0 ; i < ROW_COUNT; i++) {
            //对于当前行，设置输出为低电平，其他行全部输出高电平
            if(i == row) {
                HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, (1<<i), GPIO_PIN_RESET);
            }
            else {
                HAL_GPIO_WritePin(KEYBOARD_ROW_PORT, (1<<i), GPIO_PIN_SET);
            }
        }
        //扫描部分：然后逐列检测IO引脚电平
        for(int col = 0 ; col < COL_COUNT; col++) {
            //本来使能了上拉电阻，如果读到低电平，说明对应按键被按下
            if(HAL_GPIO_ReadPin(KEYBOARD_COL_PORT, col) == GPIO_PIN_RESET) {
                //说明这一行被按下了
                result[row*COL_COUNT+col] = 1;
            }
            else {
                result[row*COL_COUNT+col] = 0;
            }
        }
    }
}