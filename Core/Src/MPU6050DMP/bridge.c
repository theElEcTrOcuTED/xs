//
// Created by benea on 25-3-24.
//

#include "bridge.h"
//#include "i2c.h"
#include "i2c_sim.h"
//#include "i2c_sim.c"

//I2C_HandleTypeDef* hi2c1;
void mpu6050_bridge_init(I2C_HandleTypeDef* hi2c) {
 // hi2c1 = hi2c;
}

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf){
  //HAL_I2C_Mem_Write(&hi2c1, 0xD0, reg , I2C_MEMADD_SIZE_8BIT , buf, len ,100);
  I2C_Sim_Mem_Write(&i2cs1, 0xD0>>1, reg , I2C_MEMADD_SIZE_8BIT , buf, len);
  return 0;
  //return (HAL_I2C_Mem_Write(&hi2c1, 0xD0, reg , I2C_MEMADD_SIZE_8BIT , buf, len ,100) == HAL_OK)?0:-1;
}

uint8_t mpu6050_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf){
  I2C_Sim_Mem_Read(&i2cs1, 0xD0>>1, reg, I2C_MEMADD_SIZE_8BIT, buf, len);
  return 0;
  //return (HAL_I2C_Mem_Read(&hi2c1, 0xD0, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK)?0:-1;
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
  mpu6050_write_reg(PWR_MGMT_1,	 0X00);	//????MPU6050
  mpu6050_write_reg(SMPLRT_DIV,  0x07); //???????????125Hz
  mpu6050_write_reg(CONFIG,      0x06); //?????????5Hz
  mpu6050_write_reg(GYRO_CONFIG, 0x18); //???????????????¦¶:?????2000deg/s
  mpu6050_write_reg(ACCEL_CONFIG,0x01); //??????????????¦¶???????????:?????2G??5Hz
}