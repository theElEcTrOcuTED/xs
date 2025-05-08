/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "esp.h"
//#include "tb6612.h"
//#include "mpu6050.h"
#include <delay.h>
#include <math.h>

#include "ins.h"
#include "MPU6050DMP/inv_mpu.h"
#include "debug_usart.h"
#include "keyboard.h"

//#include "BLE/atk_mw579.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
MPU6050_HandleTypeDef MPU6050_hand; //MPU6050 句柄结构体
AttitudeEstimator attitudeEstimator;//MPU6050 姿态解算结构体
EulerAngle eulerAngle;//MPU6050 当前欧拉角*/

//从机ID
const int SLAVE_ID = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MasterESPDataHandler(uint8_t conn_id, uint8_t* data, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
asm(".global _printf_float");
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  DelayUs_Init();
  //keyboard_init();
 //mpu6050_bridge_init(&hi2c1);
  MPU6050_DMP_Init();
  /*
  uint8_t dat = 0x00;
  while(1) {
    HAL_I2C_Mem_Write(&hi2c1, 0b00001000, 0x0D , I2C_MEMADD_SIZE_8BIT , &dat, 1 ,100);
   // mpu6050_write(0x1F, 0x0D, 1, &dat);
  }*/
  //MPU6050_DMP_Init();
  HAL_UART_Transmit(&huart3,"123",sizeof("123"),100);
  delay_ms(1000);    //这一句有问题
  float ax,ay,az;
  HAL_UART_Transmit(&huart3,"123",sizeof("123"),100);
  MPU6050_DMP_get_accel(&ax,&ay,&az);
  HAL_UART_Transmit(&huart3,"123",sizeof("123"),100);
  ins_init(ATTITUDE_MODE_QUATERNION,0,0,0,sqrtf(ax*ax+ay*ay+az*az));





  //串口调试初始化
  debug_usart_init(6,&huart3,100);
  float number[] ={0,1,2,3};
 // HAL_UART_Transmit(&huart3,(uint8_t*)number,sizeof(float)*4,100);
  //debug_usart_send(number);
  //启用TIM2中断（100Hz）用于更新姿态


  //ESP01 串口转WIFI模块初始化
  HAL_UART_Transmit(&huart3,"ESP01 initializing",sizeof("ESP01 initializing"),100);
  ESP01_Init(&huart2,0);
  HAL_UART_Transmit(&huart3,"ESP01 initialized",sizeof("ESP01 initialized"),100);
  //设置用来处理接收到的信息的回调函数
  ESP01_SetDataCallback(MasterESPDataHandler);


  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */





//NVIC中断服务函数

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  //TIM2定时器中断，触发频率100Hz，用于更新姿态
  if(htim->Instance==TIM2) {
    //更新当前姿态
    float q0,q1,q2,q3;
    float ax,ay,az,gx,gy,gz;
    float pitch,yaw,roll;

    int a =  MPU6050_DMP_Get_Data_quaternion(&q0,&q1,&q2,&q3);
    MPU6050_DMP_get_accel(&ax,&ay,&az);
    MPU6050_DMP_get_gyro(&gx,&gy,&gz);
    //MPU6050_DMP_Get_Data_euler(&pitch,&roll,&yaw);
    pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
    roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
    yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
    ins_update_current_quaternion(q0,q1,q2,q3);
    ins_update_pos(ax,ay,az,1.0f/100);
    float posx,posy,posz;
    float vx,vy,vz;
    ins_get_position(&posx,&posy,&posz);
    ins_get_velocity(&vx,&vy,&vz);
    float data[6]={
      pitch,
      roll,
      yaw,
      q0,
      q1,
      q2,
    };
    debug_usart_send(data);//发送调试数据
    //构建数据包
    uint8_t buffer[42];
    memcpy(buffer,">HEAD",5);
    memcpy(buffer+5,&SLAVE_ID,4);
    memcpy(buffer+9,&pitch,4);
    memcpy(buffer+13,&yaw,4);
    memcpy(buffer+17,&roll,4);
    memcpy(buffer+21,&q0,4);
    memcpy(buffer+25,&q1,4);
    memcpy(buffer+29,&q2,4);
    memcpy(buffer+33,&q3,4);
    memcpy(buffer+37,">END",4);
  }

}
//自定义的主机的接+IPD消息处理函数
void MasterESPDataHandler(uint8_t conn_id, uint8_t* data, uint16_t len){
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
