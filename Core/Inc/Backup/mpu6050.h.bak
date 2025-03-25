//
// Created by benea on 25-3-5.
//



/**
* MPU6050
*/
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx.h"

// MPU6050地址定义
#define MPU6050_ADDR 0xD0 // 默认地址(AD0接GND)


// 寄存器地址定义
#define SMPLRT_DIV     0x19    //采样率分频器寄存器地址
#define CONFIG         0x1A    //配置寄存器地址
#define GYRO_CONFIG    0x1B    //陀螺仪配置寄存器地址
#define ACCEL_CONFIG   0x1C    //加速度计配置寄存器地址
#define ACCEL_XOUT_H   0x3B    //加速度计数据值寄存器地址
#define TEMP_OUT_H     0x41    //温度传感器数据值寄存器地址
#define GYRO_XOUT_H    0x43    //陀螺仪数据值寄存器地址
#define PWR_MGMT_1     0x6B    //电源管理寄存器1地址
#define WHO_AM_I       0x75    //编码寄存器地址
//以下为姿态解算部分要用到的寄存器宏定义
#define MPU6050_INTERRUPT_EN 0x12  // INT_ENABLE寄存器
#define MPU6050_INT_STATUS   0x3A  // INT_STATUS寄存器
#define MPU6050_USER_CTRL    0x6A  // USER_CTRL寄存器
#define MPU6050_FIFO_EN      0x23  // FIFO_EN寄存器
#define MPU6050_FIFO_COUNTH  0x72  // FIFO计数高字节
#define MPU6050_FIFO_R_W     0x74  // FIFO数据寄存器
#define FIFO_SAFE_THRESHOLD  (1024 - 3*28)
/**
* 加速度计最大量程枚举体
*/
enum ACC_SCALE{
  ACC_SCALE_2G = 0,
  ACC_SCALE_4G ,
  ACC_SCALE_8G ,
  ACC_SCALE_16G
  };
/**
* 陀螺仪最大量程枚举体
*/
enum GYRO_SCALE{
  GYRO_SCALE_250_DPS = 0,
  GYRO_SCALE_500_DPS ,
  GYRO_SCALE_1000_DPS ,
  GYRO_SCALE_2000_DPS
};




// 基本数据结构体（不包括姿态解算结果）
typedef struct {
  float Accel_X;  // 加速度计X轴 (g)
  float Accel_Y;
  float Accel_Z;
  float Gyro_X;   // 陀螺仪X轴 (°/s)
  float Gyro_Y;
  float Gyro_Z;
  float Temp;     // 温度 (℃)
} MPU6050_Data;




/**
* 连接到I2C总线的MPU6050结构体
*/
typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t Address;
  enum ACC_SCALE aScale;
  enum GYRO_SCALE gScale;
  int16_t Accel_Offset[3];
  int16_t Gyro_Offset[3];
  //以下为DMP相关的成员
  uint8_t dmpReady;       // DMP初始化状态
  uint16_t packetSize;    // DMP数据包大小
  void (*DMP_TapCallback)(uint8_t direction, uint8_t count); // 点击回调
} MPU6050_HandleTypeDef;


// 函数声明
uint8_t myMPU6050_Init(MPU6050_HandleTypeDef *hmpu, I2C_HandleTypeDef *hi2c,
                    enum ACC_SCALE aScl, enum GYRO_SCALE gScl);
void MPU6050_ReadRawData(MPU6050_HandleTypeDef *hmpu, int16_t *accel, int16_t *gyro, int16_t *temp);
void MPU6050_ReadProcessedData(MPU6050_HandleTypeDef *hmpu, MPU6050_Data *data);
void MPU6050_Calibrate(MPU6050_HandleTypeDef *hmpu, uint16_t sampleCount);
HAL_StatusTypeDef MPU6050_TestConnection(MPU6050_HandleTypeDef *hmpu);


















//以下为姿态解算部分



// 新增DMP相关数据结构
typedef struct {
  float pitch;
  float roll;
  float yaw;
} DMP_Euler_Angles;

typedef struct {
  float w;
  float x;
  float y;
  float z;
} DMP_Quaternion;



// 新增函数声明
uint8_t MPU6050_DMP_Init(MPU6050_HandleTypeDef *hmpu);
uint8_t MPU6050_DMP_GetData(MPU6050_HandleTypeDef *hmpu, DMP_Quaternion *q, DMP_Euler_Angles *euler);
void MPU6050_DMP_ResetFIFO(MPU6050_HandleTypeDef *hmpu);


//软件姿态解算部分
// 配置参数

#define ALPHA_LPF       0.15f     // 加速度计低通滤波系数
#define ALPHA_HPF       0.98f     // 陀螺仪高通滤波系数
#define BETA_COMP       0.1f      // 互补滤波系数
typedef struct {
  float q[4];          // 四元数
  float inte_err[3];    // 积分误差
  float gyro_bias[3];   // 陀螺仪零偏
  float gyro_lpf[3];   // 陀螺仪低通滤波状态
  float acc_lpf[3];    // 加速度计低通滤波状态
} AttitudeEstimator;

typedef struct {
  float pitch;
  float roll;
  float yaw;
} EulerAngle;
void MPU6050_init_estimator(AttitudeEstimator* est,MPU6050_Data *data);

void MPU6050_get_euler_angles(const AttitudeEstimator* est,
                     float* roll, float* pitch, float* yaw);
float MPU6050_getKP();
float MPU6050_getKI();
float MPU6050_getDT();
float MPU6050_setKP(float);
float MPU6050_setKI(float);
float MPU6050_setDT(float);






void MPU6050_update_attitude(EulerAngle *output,float ax, float ay, float az,
                    float gx, float gy, float gz);
void MPU6050_MyCalibrate(MPU6050_HandleTypeDef *hmpu);
//[DEBUG]实验性函数
EulerAngle MPU6050_GetKalmanAngle();









#endif /* MPU6050_H */




