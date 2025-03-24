//
// Created by benea on 25-3-24.
//

#include "bridge.h"

I2C_HandleTypeDef hi2c1;

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf){
  return (HAL_I2C_Mem_Write(&hi2c1, addr, reg , I2C_MEMADD_SIZE_8BIT , buf, len ,100) == HAL_OK)?0:-1;
}

uint8_t mpu6050_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf){
  return (HAL_I2C_Mem_Read(&hi2c1, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK)?0:-1;
}
void mpu6050_write_reg(uint8_t reg, uint8_t dat)
{
  mpu6050_write(SlaveAddress,reg,8,&dat);
}

uint8_t mpu6050_read_reg (uint8_t reg)
    {
  uint8_t dat;
  mpu6050_read(SlaveAddress, reg, 1, &dat);
  return dat;
  }

void MPU6050_Init(void)
{
  mpu6050_write_reg(PWR_MGMT_1,	 0X00);	//唤醒MPU6050
  mpu6050_write_reg(SMPLRT_DIV,  0x07); //陀螺仪采样率125Hz
  mpu6050_write_reg(CONFIG,      0x06); //低通滤波频率5Hz
  mpu6050_write_reg(GYRO_CONFIG, 0x18); //陀螺仪自检及测量范围:不自检，2000deg/s
  mpu6050_write_reg(ACCEL_CONFIG,0x01); //加速计自检、测量范围及高通滤波频率:不自检，2G，5Hz
}