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
#include "mpu6050.h"
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
MPU6050_HandleTypeDef MPU6050_hand; //MPU6050 句柄结构体
AttitudeEstimator attitudeEstimator;//MPU6050 姿态解算结构体
EulerAngle eulerAngle;//MPU6050 当前欧拉角
Motor_HandleTypeDef Motor_Handle;
TB6612_HandleTypeDef TB6612_Handle;
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
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&MPU6050_hand,&hi2c1,ACC_SCALE_2G,GYRO_SCALE_250_DPS);//MPU6050初始化



  //等待MPU6050初始化
  HAL_Delay(150);
  //姿态角控制目标初始化
  MPU6050_Data predata;
  MPU6050_ReadProcessedData(&MPU6050_hand,&predata);
  MPU6050_init_estimator(&attitudeEstimator,&predata);//MPU6050 软件姿态解算初始化
  MPU6050_update_attitude(&attitudeEstimator,predata.Accel_X,predata.Accel_Y,predata.Accel_Z,predata.Gyro_X,predata.Gyro_Y,predata.Gyro_Z);
  //欧拉角解算
  MPU6050_get_euler_angles(&attitudeEstimator,&eulerAngle.roll,&eulerAngle.pitch,&eulerAngle.yaw);

  controlTarget = eulerAngle.pitch;


  /*
  Motor_Handle.in1_pin = GPIO_PIN_14;
  Motor_Handle.in2_pin = GPIO_PIN_13;
  Motor_Handle.in1_port = GPIOB;
  Motor_Handle.in2_port = GPIOB;
  Motor_Handle.pwm_ch = TIM_CHANNEL_1;
  Motor_Handle.pwm_tim = &htim1;*/
  Motor_Init(&Motor_Handle,&htim1,TIM_CHANNEL_1,GPIOB,GPIO_PIN_14,GPIOB,GPIO_PIN_13);
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
  TB6612_Init(&TB6612_Handle,&Motor_Handle,NULL,NULL,0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 70);
  TB6612_Enable(&TB6612_Handle);
  HAL_TIM_Base_Start_IT(&htim2);
 // Motor_SetSpeed(&TB6612_Handle.motorA, 20);
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

double Kp = 2.0;
double Ki = 0.01;
double Kd = 0.1;
double dt = 1/500.0f;
double inte_error;
double last_error;



//NVIC中断服务函数

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {

  if(htim->Instance==TIM2) {
    //TIM2 PSC = 63, ARR = 2000, 500Hz，用于触发MPU6050采样和姿态解算

    MPU6050_Data predata;
    MPU6050_ReadProcessedData(&MPU6050_hand,&predata);

    MPU6050_update_attitude(&attitudeEstimator,predata.Accel_X,predata.Accel_Y,predata.Accel_Z,predata.Gyro_X,predata.Gyro_Y,predata.Gyro_Z);

    //欧拉角解算
    MPU6050_get_euler_angles(&attitudeEstimator,&eulerAngle.roll,&eulerAngle.pitch,&eulerAngle.yaw);

    //TIM1 advanced - control timer:
    //PSC = 63 -->> 1MHz,  ARR = 100 -> 10KHz PWM波，默认Pulse = 75, 上计数模式，PWM Mode1,占空比75%

    //pid控制
    //受控变量：俯仰角
    double error = eulerAngle.pitch - controlTarget;            //当前误差
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

    //pitch以抬头为正，设风扇正向下吹，则pitch升高时风扇转速应降低，即CCR值应降低
    char message[70] = "pitch=";
    char pitchStr[7];
    char axStr[7];
    char ayStr[7];
    char azStr[7];
    char gyroxStr[7];
    char gyroyStr[7];
    char gyrozStr[7];
    sprintf(pitchStr,"%.2lf",eulerAngle.pitch);
    sprintf(axStr,"%.2lf",predata.Accel_X);
    sprintf(ayStr,"%.2lf",predata.Accel_Y);
    sprintf(azStr,"%.2lf",predata.Accel_Z);
    sprintf(gyroxStr,"%.2lf",predata.Gyro_X);
    sprintf(gyroyStr,"%.2lf",predata.Gyro_Y);
    sprintf(gyrozStr,"%.2lf",predata.Gyro_Z);
    strcat(message,pitchStr);
    strcat(message,"\nax=");
    strcat(message,axStr);
    strcat(message,"\nay=");
    strcat(message,ayStr);
    strcat(message,"\naz=");
    strcat(message,azStr);
    strcat(message,"\ngx=");
    strcat(message,gyroxStr);
    strcat(message,"\ngy=");
    strcat(message,gyroyStr);
    strcat(message,"\ngz=");
    strcat(message,gyrozStr);
    strcat(message,"\n\n");
    HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), 100);


    //更新风扇转速
    Motor_SetSpeed(&TB6612_Handle.motorA, output*100);
    //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 20);
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
