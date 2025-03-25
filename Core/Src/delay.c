/* delay.c */
#include "delay.h"

TIM_HandleTypeDef htim_delay;

void DelayUs_Init(void) {
    /* 使能TIM时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();  // 使用TIM6，可根据需求更换其他定时器

    /* 计算定时器时钟 */
    RCC_ClkInitTypeDef clk_config;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk_config, &flash_latency);

    uint32_t apb1_clock = HAL_RCC_GetPCLK1Freq();
    /* 检查APB1预分频系数 */
    if (clk_config.APB1CLKDivider != RCC_HCLK_DIV1) {
        apb1_clock *= 2;  // 当APB1分频系数>1时，定时器时钟2倍频
    }

    /* 配置预分频器使定时器频率=1MHz */
    uint16_t prescaler = (apb1_clock / 1000000) - 1;

    /* 初始化定时器 */
    htim_delay.Instance = TIM4;
    htim_delay.Init.Prescaler = prescaler;
    htim_delay.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_delay.Init.Period = 0xFFFF;  // 最大周期值
    htim_delay.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim_delay) != HAL_OK) {
       /* Error_Handler();  // 用户需实现错误处理*/
    }

    HAL_TIM_Base_Start(&htim_delay);  // 启动定时器
}

void delay_us(uint16_t us) {
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim_delay);
    uint16_t arr = __HAL_TIM_GetAutoreload(&htim_delay);
    if(arr-start>us) {
        int count = us / (arr-start);
        for(int i = 0 ; i < count ; i++) {
            while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim_delay)) - start < us);
        }
    }
    else {
        while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim_delay)) - start < us);
    }
}
void delay_ms(uint32_t ms) {
  delay_us(ms * 1000);
}
