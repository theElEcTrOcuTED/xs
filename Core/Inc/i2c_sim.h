//
// Created by benea on 25-5-9.
//

#ifndef I2C_SIM_H
#define I2C_SIM_H
/* 模拟I2C驱动头文件 (i2c_sim.h) */
#include "stm32f1xx_hal.h"
#include "delay.h"
typedef struct {
    GPIO_TypeDef *SDA_GPIOx;
    uint16_t SDA_GPIO_Pin;
    GPIO_TypeDef *SCL_GPIOx;
    uint16_t SCL_GPIO_Pin;
    uint32_t Timeout;  // 超时时间（单位：ms）
} I2C_Sim_HandleTypeDef;

/* 函数声明 */
void I2C_Sim_Init(I2C_Sim_HandleTypeDef *hi2c);
HAL_StatusTypeDef I2C_Sim_Mem_Write(I2C_Sim_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef I2C_Sim_Mem_Read(I2C_Sim_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

/* 私有函数声明 */
static void I2C_Sim_Start(I2C_Sim_HandleTypeDef *hi2c);
static void I2C_Sim_Stop(I2C_Sim_HandleTypeDef *hi2c);
static HAL_StatusTypeDef I2C_Sim_Wait_Ack(I2C_Sim_HandleTypeDef *hi2c);
static void I2C_Sim_Ack(I2C_Sim_HandleTypeDef *hi2c);
static void I2C_Sim_NAck(I2C_Sim_HandleTypeDef *hi2c);
static HAL_StatusTypeDef I2C_Sim_WriteByte(I2C_Sim_HandleTypeDef *hi2c, uint8_t data);
static uint8_t I2C_Sim_ReadByte(I2C_Sim_HandleTypeDef *hi2c);
#endif //I2C_SIM_H
