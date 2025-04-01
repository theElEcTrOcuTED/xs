/* delay.h */
#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"  // 根据实际芯片型号修改头文件
#include "stm32f1xx.h"
    void DelayUs_Init(void);
    void delay_us(uint32_t us);
    void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */