#ifndef TB6612_H
#define TB6612_H

#include "stm32f1xx_hal.h"

// 电机方向枚举
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE,
    MOTOR_STOP
} Motor_Direction;

// 电机结构体
typedef struct {
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_ch;
    GPIO_TypeDef* in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef* in2_port;
    uint16_t in2_pin;
} Motor_HandleTypeDef;

// TB6612驱动板结构体
typedef struct {
    Motor_HandleTypeDef motorA;
    Motor_HandleTypeDef motorB;
    GPIO_TypeDef* stby_port;
    uint16_t stby_pin;
} TB6612_HandleTypeDef;

// 电机初始化函数
void Motor_Init(Motor_HandleTypeDef* motor, 
               TIM_HandleTypeDef* pwm_tim,
               uint32_t pwm_ch,
               GPIO_TypeDef* in1_port,
               uint16_t in1_pin,
               GPIO_TypeDef* in2_port,
               uint16_t in2_pin);

void Motor_SetSpeed(Motor_HandleTypeDef* motor, float speed);
void Motor_SetDirection(Motor_HandleTypeDef* motor, Motor_Direction dir);
void Motor_Brake(Motor_HandleTypeDef* motor);
void Motor_Stop(Motor_HandleTypeDef* motor);

// TB6612驱动板控制函数
void TB6612_Init(TB6612_HandleTypeDef* driver,
                Motor_HandleTypeDef* motorA,
                Motor_HandleTypeDef* motorB,
                GPIO_TypeDef* stby_port,
                uint16_t stby_pin);
void TB6612_Enable(TB6612_HandleTypeDef* driver);
void TB6612_Disable(TB6612_HandleTypeDef* driver);

#endif