/* delay.c */
#include "delay.h"

TIM_HandleTypeDef htim_delay;
static uint8_t  fac_us=0;							//us延时倍乘数
static uint16_t fac_ms=0;

void DelayUs_Init(void) {
    /* 使能TIM时钟 */
    __HAL_RCC_TIM4_CLK_ENABLE();  // 使用TIM4，可根据需求更换其他定时器

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



    //SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8

    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);//选择SysTick的时钟信号源为HCLK/8的八分频
    fac_us=HAL_RCC_GetHCLKFreq()/8000000;				//为系统时钟的1/8
    fac_ms=(uint16_t)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数
}

void delay_us(uint32_t us) {

    /*
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
    //ARR设为要延时的微秒数
    __HAL_TIM_DISABLE(&htim_delay);
    __HAL_TIM_SET_AUTORELOAD(&htim_delay, us);
    //重置计数器
    __HAL_TIM_SET_COUNTER(&htim_delay, 0);
    __HAL_TIM_ENABLE(&htim_delay);
    do {
    }while(__HAL_TIM_GET_COUNTER(&htim_delay) != 0);*/
    uint32_t temp;
    SysTick->LOAD=us*fac_us; 					//时间加载
    SysTick->VAL=0x00;        					//清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
    SysTick->VAL =0X00;      					 //清空计数器

}
void delay_ms(uint32_t ms) {
    uint32_t temp;
    SysTick->LOAD=(uint32_t)ms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
    SysTick->VAL =0x00;							//清空计数器
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数
    do
    {
        temp=SysTick->CTRL;
    }while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
    SysTick->VAL =0X00;       					//清空计数器
}
