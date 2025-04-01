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

#include "tb6612.h"
//#include "mpu6050.h"
#include <delay.h>
#include <math.h>

#include "ins.h"
#include "MPU6050DMP/inv_mpu.h"
#include "debug_usart.h"
#include "keyboard.h"
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
TB6612_HandleTypeDef TB6612_Handle1;
Motor_HandleTypeDef Motor_Handle1;
Motor_HandleTypeDef Motor_Handle3;

TB6612_HandleTypeDef TB6612_Handle2;
Motor_HandleTypeDef Motor_Handle2;
Motor_HandleTypeDef Motor_Handle4;

double controlTarget;//控制目标
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  DelayUs_Init();
  /* USER CODE BEGIN 2 */
  keyboard_init();
 //mpu6050_bridge_init(&hi2c1);
  MPU6050_DMP_Init();
  /*
  uint8_t dat = 0x00;
  while(1) {
    HAL_I2C_Mem_Write(&hi2c1, 0b00001000, 0x0D , I2C_MEMADD_SIZE_8BIT , &dat, 1 ,100);
   // mpu6050_write(0x1F, 0x0D, 1, &dat);
  }*/
  //MPU6050_DMP_Init();


  HAL_Delay(1000);
  float ax,ay,az;
  MPU6050_DMP_get_accel(&ax,&ay,&az);
  ins_init(ATTITUDE_MODE_QUATERNION,0,0,0,sqrtf(ax*ax+ay*ay+az*az));



  //TB6612FNG板1、电机1,3初始化，电机1、3公用PWM IN1 IN2三个引脚
  Motor_Init(&Motor_Handle1,&htim1,TIM_CHANNEL_1,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_13);
  Motor_Init(&Motor_Handle3,&htim1,TIM_CHANNEL_1,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_13);
  TB6612_Init(&TB6612_Handle1,&Motor_Handle1,&Motor_Handle3,NULL,0);
  TB6612_Enable(&TB6612_Handle1);
  //TB6612FNG板2、电机2,4初始化，电机2、4公用PWM IN1 IN2三个引脚
  Motor_Init(&Motor_Handle2,&htim1,TIM_CHANNEL_2,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15);
  Motor_Init(&Motor_Handle4,&htim1,TIM_CHANNEL_2,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_15);
  TB6612_Init(&TB6612_Handle2,&Motor_Handle2,&Motor_Handle4,NULL,0);
  TB6612_Enable(&TB6612_Handle2);

  //串口调试初始化
  debug_usart_init(6,&huart3,5000);
  float number[] ={0,1,2,3};
 // HAL_UART_Transmit(&huart3,(uint8_t*)number,sizeof(float)*4,100);
  //debug_usart_send(number);
  //启用TIM2中断（100Hz）用于更新姿态
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

//标志当前工作状态；画直线还是圆
typedef enum {
  STRAIGHT_LINE_1,//稳定画出一条长度不短于50cm的直线段
  STRAIGHT_LINE_2,//稳定画出长度在30-60cm间可设置的直线段
  STRAIGHT_LINE_3,//按照设定的方向角度摆动，画出不短于20cm的直线段
  STOP,//拉起一定角度放开，5s内静止
  CIRCLE,
} currentStatus;


double Kp = 2.0;
double Ki = 0.01;
double Kd = 0.1;
double dt = 1/500.0f;
double inte_error;
double last_error;



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
    MPU6050_DMP_Get_Data_euler(&pitch,&roll,&yaw);
    ins_update_current_quaternion(q0,q1,q2,q3);
    ins_update_pos(ax,ay,az,1.0f/100);
    float posx,posy,posz;
    float vx,vy,vz;
    ins_get_position(&posx,&posy,&posz);
    ins_get_velocity(&vx,&vy,&vz);
    float data[6]={
      a,
      q1,
      q2,
      posx,
      posy,
      posz,
    };
    debug_usart_send(data);//发送调试数据

    //更新控制目标


    double error = 0;//eulerAngle.pitch - controlTarget;            //当前误差
    double dif = (error - last_error)/dt;       //微分量
    double output = Kp*error + Ki*inte_error + Kd*dif;//PID控制量计算
    output /= 200;
    if(output > 1.0) {
      output = 1.0;
    }
    else if (output<-1.0) {
      output = -1.0;
    }
    inte_error += error*dt;

  }
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
