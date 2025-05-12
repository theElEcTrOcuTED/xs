//
// Created by benea on 25-5-9.
//

#include "i2c_sim.h"


/* 初始化GPIO */
/* 初始化GPIO */
void I2C_Sim_Init(I2C_Sim_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* SDA GPIO初始化 */
    GPIO_InitStruct.Pin = hi2c->SDA_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(hi2c->SDA_GPIOx, &GPIO_InitStruct);

    /* SCL GPIO初始化 */
    GPIO_InitStruct.Pin = hi2c->SCL_GPIO_Pin;
    HAL_GPIO_Init(hi2c->SCL_GPIOx, &GPIO_InitStruct);

    /* 初始状态拉高总线 */
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
}

/* 模拟I2C存储写入 */
HAL_StatusTypeDef I2C_Sim_Mem_Write(I2C_Sim_HandleTypeDef *hi2c, uint16_t DevAddress,
                                  uint16_t MemAddress, uint16_t MemAddSize,
                                  uint8_t *pData, uint16_t Size) {
    HAL_StatusTypeDef status;

    I2C_Sim_Start(hi2c);
    status = I2C_Sim_WriteByte(hi2c, DevAddress << 1);
    if(status != HAL_OK) return status;

    /* 发送内存地址 */
    if(MemAddSize == I2C_MEMADD_SIZE_16BIT) {
        status = I2C_Sim_WriteByte(hi2c, (uint8_t)(MemAddress >> 8));
        if(status != HAL_OK) return status;
    }
    status = I2C_Sim_WriteByte(hi2c, (uint8_t)(MemAddress & 0xFF));
    if(status != HAL_OK) return status;

    /* 写入数据 */
    for(uint16_t i = 0; i < Size; i++) {
        status = I2C_Sim_WriteByte(hi2c, pData[i]);
        if(status != HAL_OK) return status;
    }

    I2C_Sim_Stop(hi2c);
    return HAL_OK;
}

/* 模拟I2C存储读取 */
HAL_StatusTypeDef I2C_Sim_Mem_Read(I2C_Sim_HandleTypeDef *hi2c, uint16_t DevAddress,
                                 uint16_t MemAddress, uint16_t MemAddSize,
                                 uint8_t *pData, uint16_t Size) {
    HAL_StatusTypeDef status;

    /* 先发送内存地址 */
    I2C_Sim_Start(hi2c);
    status = I2C_Sim_WriteByte(hi2c, DevAddress << 1);
    if(status != HAL_OK) return status;

    if(MemAddSize == I2C_MEMADD_SIZE_16BIT) {
        status = I2C_Sim_WriteByte(hi2c, (uint8_t)(MemAddress >> 8));
        if(status != HAL_OK) return status;
    }
    status = I2C_Sim_WriteByte(hi2c, (uint8_t)(MemAddress & 0xFF));
    if(status != HAL_OK) return status;

    /* 重新启动进行读取 */
    I2C_Sim_Start(hi2c);
    status = I2C_Sim_WriteByte(hi2c, (DevAddress << 1) | 0x01);
    if(status != HAL_OK) return status;

    /* 读取数据 */
    for(uint16_t i = 0; i < Size; i++) {
        pData[i] = I2C_Sim_ReadByte(hi2c);
        if(i == Size - 1) {
            I2C_Sim_NAck(hi2c);
        } else {
            I2C_Sim_Ack(hi2c);
        }
    }

    I2C_Sim_Stop(hi2c);
    return HAL_OK;
}

/******************** 私有函数实现 ********************/
static void I2C_Sim_Start(I2C_Sim_HandleTypeDef *hi2c) {
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
}

static void I2C_Sim_Stop(I2C_Sim_HandleTypeDef *hi2c) {
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
}

static HAL_StatusTypeDef I2C_Sim_Wait_Ack(I2C_Sim_HandleTypeDef *hi2c) {
    uint32_t wait_time = 0;

    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);

    /* 带超时的ACK检测 */
    while(HAL_GPIO_ReadPin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin)) {
        delay_us(100);
        wait_time += 100;
        if(wait_time > (hi2c->Timeout * 1000)) {  // 转换为微秒
            return HAL_TIMEOUT;
        }
    }

    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
    return HAL_OK;
}

static void I2C_Sim_Ack(I2C_Sim_HandleTypeDef *hi2c) {
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);
}

static void I2C_Sim_NAck(I2C_Sim_HandleTypeDef *hi2c) {
    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_RESET);
    delay_us(1);
}

static HAL_StatusTypeDef I2C_Sim_WriteByte(I2C_Sim_HandleTypeDef *hi2c, uint8_t data) {
    for(uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin,
                         (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1;
        delay_us(1);
        HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
        delay_us(1);
        HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_RESET);
        delay_us(1);
    }
    return I2C_Sim_Wait_Ack(hi2c);
}

static uint8_t I2C_Sim_ReadByte(I2C_Sim_HandleTypeDef *hi2c) {
    uint8_t data = 0;

    HAL_GPIO_WritePin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin, GPIO_PIN_SET);

    for(uint8_t i = 0; i < 8; i++) {
        data <<= 1;
        HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_SET);
        delay_us(1);
        if(HAL_GPIO_ReadPin(hi2c->SDA_GPIOx, hi2c->SDA_GPIO_Pin)) data |= 0x01;
        HAL_GPIO_WritePin(hi2c->SCL_GPIOx, hi2c->SCL_GPIO_Pin, GPIO_PIN_RESET);
        delay_us(1);
    }
    return data;
}