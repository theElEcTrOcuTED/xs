#include "tb6612.h"

void Motor_Init(Motor_HandleTypeDef* motor, 
               TIM_HandleTypeDef* pwm_tim,
               uint32_t pwm_ch,
               GPIO_TypeDef* in1_port,
               uint16_t in1_pin,
               GPIO_TypeDef* in2_port,
               uint16_t in2_pin)
{
    motor->pwm_tim = pwm_tim;
    motor->pwm_ch = pwm_ch;
    motor->in1_port = in1_port;
    motor->in1_pin = in1_pin;
    motor->in2_port = in2_port;
    motor->in2_pin = in2_pin;

    // 初始化方向引脚
    GPIO_InitTypeDef gpio_init = {
        .Pin = in1_pin,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH
    };
    HAL_GPIO_Init(in1_port, &gpio_init);

    gpio_init.Pin = in2_pin;
    HAL_GPIO_Init(in2_port, &gpio_init);

    // 启动PWM
    HAL_TIM_PWM_Start(pwm_tim, pwm_ch);
}

/**
 * 当速度为0时，自动设置为滑行（快衰减）而不是刹车。如要刹车，应调用Motor_SetDirection并设置为BRAKE（即慢衰减）.
 * @param motor
 * @param speed
 */
void Motor_SetSpeed(Motor_HandleTypeDef* motor, float speed)
{
    // 限制速度范围
    speed = (speed > 100) ? 100 : (speed < -100) ? -100 : speed;


    if(speed > 0) {
        Motor_SetDirection(motor, MOTOR_FORWARD);
    } else if(speed < 0) {
        Motor_SetDirection(motor, MOTOR_BACKWARD);
        speed = -speed;
    } else {
        Motor_Stop(motor);
        return;
    }
    // 计算PWM占空比
    if(speed!=0) {
        uint32_t arr = motor->pwm_tim->Instance->ARR;
        __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_ch, (int)(speed * arr) / 100);
    }
}

void Motor_SetDirection(Motor_HandleTypeDef* motor, Motor_Direction dir)
{
    if(motor) {
        switch(dir) {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            break;
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            break;
            case MOTOR_BRAKE:
                HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            break;
            case MOTOR_STOP:
                HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            break;
        }
    }
}

void Motor_Brake(Motor_HandleTypeDef* motor)
{
    Motor_SetDirection(motor, MOTOR_BRAKE);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_ch, 0);
}

void Motor_Stop(Motor_HandleTypeDef* motor)
{
    Motor_SetDirection(motor, MOTOR_STOP);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_ch, 0);
}

/******************** TB6612驱动板相关函数 ********************/
void TB6612_Init(TB6612_HandleTypeDef* driver,
                Motor_HandleTypeDef* motorA,
                Motor_HandleTypeDef* motorB,
                GPIO_TypeDef* stby_port,
                uint16_t stby_pin)
{
    // 复制电机配置
    if(motorA) {
        driver->motorA = *motorA;
    }
    if(motorB) {
        driver->motorB = *motorB;
    }
    driver->stby_port = stby_port;
    driver->stby_pin = stby_pin;
    if(stby_port != NULL) {
        // 初始化STBY引脚
        GPIO_InitTypeDef gpio_init = {
            .Pin = stby_pin,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH
        };
        HAL_GPIO_Init(stby_port, &gpio_init);
    }
    // 默认禁用驱动板
    TB6612_Disable(driver);
}

void TB6612_Enable(TB6612_HandleTypeDef* driver)
{
    if(driver->stby_port != NULL) {
        HAL_GPIO_WritePin(driver->stby_port, driver->stby_pin, GPIO_PIN_SET);
    }
}

void TB6612_Disable(TB6612_HandleTypeDef* driver)
{
    if(driver->stby_port != NULL) {
        HAL_GPIO_WritePin(driver->stby_port, driver->stby_pin, GPIO_PIN_RESET);
    }

}