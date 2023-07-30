/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Proportional_Factor=1,Integral_Factor=0.03,Derivative_Factor=0.1;
float Target_Angle = 0;
float output,input,Angle;
float Proportional=0, Integral=0, Derivative=0;
float PID;
float Angle_Error;
float Previous_Error = 0;


int flag;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
MPU6050_t MPU6050;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint16_t map(long x,long in_min,long in_max,long out_min,long out_max);
void MX_GPIO_Init(void);
void MX_I2C2_Init(void);
void MX_USART2_UART_Init(void);
float ComputePID(float input, float Target_Angle);
/* USER CODE BEGIN PFP */
void Servo_SetAngle(float Angle);
void PWM_SetCompare2(uint16_t Compare);
/* USER CODE END PFP */
void motor();

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
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

  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */




       /* Infinite loop */
       /* USER CODE BEGIN WHILE */
 MX_USART2_UART_Init();
 HAL_Delay (1000);
 while (MPU6050_Init(&hi2c2) == 1);
   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   while (1)
   {
 	  MPU6050_Read_All(&hi2c2, &MPU6050);

 	 	 	 input=MPU6050.KalmanAngleX;
 	 	 	 /* if(input<Target_Angle)
 	 	 	 {
 	 	 		output=ComputePID( input, Target_Angle)+90;
 	 	 	 }
 	 	 	 else if(input>Target_Angle)
 	 	 	 {
 	 	 		output=90-ComputePID( input, Target_Angle);
 	 	 	 }*/
 	 	 	output=ComputePID( input, Target_Angle)+90;
 	 	 	Angle=map(output,7,170,20,160);
 	 	 	if(Angle>160)
 	 	 	{
 	 	 		Angle=160;
 	 	 	}
 	 	 	if(Angle<20)
 	 	 	{
 	 	 		Angle=20;
 	 	 	}

 	 	 	 printf("%f\t\n",Angle);
   }
   /* USER CODE END 3 */
 }

uint16_t map(long x,long in_min,long in_max,long out_min,long out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
float ComputePID(float  input, float Target_Angle)
{

    Angle_Error = Target_Angle-input;
    Proportional = Proportional_Factor * Angle_Error;


    if( -5< Angle_Error && Angle_Error < 5)
    {
    	Integral = Integral + (Integral_Factor * Angle_Error);
    	flag=1;
    }
    else
    {
    	flag=0;
    }
    Derivative = Derivative_Factor * (Angle_Error - Previous_Error) ;
   output = Proportional + Integral*flag+ Derivative;

    Previous_Error = Angle_Error;
    return output;
}
void motor()
{
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	 htim3.Instance->CCR1 = 200;
	  		   	 	  HAL_Delay(4000);
	  		   	       htim3.Instance->CCR1 =100;
	  		   	       HAL_Delay(4000);
	  		   	  htim3.Instance->CCR1 = 190;
	  		   	   		   	    	   	            HAL_Delay(10000);

}

     void PWM_SetCompare2(uint16_t Compare)
     {
    	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
     	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Compare);
     	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Compare);
     }
     void Servo_SetAngle(float Angle)
     {
     	PWM_SetCompare2(Angle/180*200+50);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
