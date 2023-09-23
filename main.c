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
#include "gpio.h"
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include "usart.h"
float Proportional_Factor=0.5,Integral_Factor=0.005,Derivative_Factor=0.1;
float Target_Angle =0 ;
float Target_Angle2 = 0;
float output,output1,input,Roll,Roll1,Angle,Angle1,output2;
float Proportional=0, Integral=0, Derivative=0;
float PID;
float Angle_Error;
float Previous_Error = 0;
#define dt 0.005
#define mid 90
int flag;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_t MPU6050;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t map(long x,long in_min,long in_max,long out_min,long out_max);
void MX_GPIO_Init(void);
void MX_I2C2_Init(void);
void MX_USART1_UART_Init(void);
float ComputePID(float input, float Target_Anglex);
/* USER CODE BEGIN PFP */
void Servo_SetAngle(float Angl,float Ang);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare1(uint16_t Compar);
/* USER CODE END PFP */
void motor();
/* USER CODE END PFP */
float angle1();
float roll1();
void motormin();
void motormax();
void RollServo_SetRoll(float Rol,float Ro);

void PitchServo_SetAnglex(float Angl);
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
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
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  motor();
  PitchServo_SetAnglex(mid);
  /* USER CODE BEGIN 2 */
  MPU6050_Init(&hi2c2);
      /* USER CODE END 2 */
   HAL_Delay (1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_2))
	 	 	  	 {
		  if(HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
		  {
			  motormin();
		  }
	 	 	  	  //Turn LED ON
	 		  motormax();
	 		  HAL_Delay(6000);
	 		 motormin();
	 	 	  	  }
    /* USER CODE END WHILE */
	  HAL_Delay (50);
	 		  MPU6050_Read_All(&hi2c2, &MPU6050);



	 	 	 	 	 /* if(input<Target_Angle)
	 	 	 	 	 {
	 	 	 	 		output=ComputePID( input, Target_Angle)+90;
	 	 	 	 	 }
	 	 	 	 	 else if(input>Target_Angle)
	 	 	 	 	 {
	 	 	 	 		output=90-ComputePID( input, Target_Angle);
	 	 	 	 	 }*/
	 	 	 	 	output1=(ComputePID( MPU6050.KalmanAngleX, Target_Angle)+90);
	 	 	 	 	output2=(ComputePID( MPU6050.KalmanAngleY, Target_Angle2)+90);
	 	 	 	 	Angle=map(output2,57,125,10,170);
	 	 	 		Roll=map(output1,57,125,10,170);
	 	 	 	 	if(Angle>160)
	 	 	 	 	{
	 	 	 	 		Angle=160;
	 	 	 	 	}
	 	 	 	 	if(Angle<20)
	 	 	 	 	{
	 	 	 	 		Angle=20;
	 	 	 	 	}
	 	 	 	 	if(Roll>160)
	 	 	 	 		 	 	 	{
	 	 	 	 		Roll=160;
	 	 	 	 		 	 	 	}
	 	 	 	 		 	 	 	if(Roll<20)
	 	 	 	 		 	 	 	{
	 	 	 	 		 	 	 	Roll=20;
	 	 	 	 		 	 	 	}
	 	 	 	 	angle1(Angle);
	 	 	 	 	roll1(Roll);
	 	 	 	 /* Servo_SetRoll(Roll);*/
	 	 	 	 /* HAL_Delay (450);*/




	 	 	 	 	/*  Servo_SetAngle(Angle,Angle1);*/
	 	 	 	 	/* HAL_Delay (10);*/
	 	 	 	 	/* printf("%f\t\n",Angle1);*/
	 	 	 	 /* Servo_SetAngle(Angle1,Angle);*/
	 	 	 	 	/*  if (Roll>90)
	 	 	 	 	{
	 	 	 	 		RollServo_SetRollright(Roll);
	 	 	 	 	}
	 	 	 	 	if (Roll<90)
	 	 	 	 		 	 	 	{
	 	 	 	 		 	 	 		Servo_SetRollleft(Roll1);
	 	 	 	 		 	 	 	}*/

	 	 	 if(fabs(MPU6050.KalmanAngleX)>10)
	 	 	 	 		 	 	 		 	 	 	{

	 	 	 	 		 	 	 		 	 	RollServo_SetRoll( Roll, Roll1);
	 	 	 	 		 	 	 		 	 	 	}

	 	 	 	 	if(fabs(MPU6050.KalmanAngleX)<=10)
	 	 	 	 	{
	 	 	 	 	 PitchServo_SetAnglex(Angle1);

	 	 	 		 	 	 	 		 	 	 		 	 	}

	 	 	 	 	 printf("%f\t\n",Angle1);
	 	 	 	  printf("%f\t\n",Angle);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
float angle1()
{
	if(Angle>90)
		 	 	 	{
		 	 	 		Angle1=160-Angle+20;
		 	 	 	}
		 	 		if(Angle<90)
		 	 		{
		 	 			Angle1=160-(Angle-20);
		 	 		}
		 	 		if(Angle==90)
		 	 		{
		 	 			Angle1=90;
		 	 		}
		 	 		return Angle1;
}
float roll1()
{
	if(Roll>90)
		 	 	 	{
		Roll1=160-Roll+20;
		 	 	 	}
		 	 		if(Roll<90)
		 	 		{
		 	 			Roll1=160-(Roll-20);
		 	 		}
		 	 		if(Roll==90)
		 	 		{
		 	 			Roll1=90;
		 	 		}
		 	 		return Roll1;
}
uint16_t map(long x,long in_min,long in_max,long out_min,long out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}
float ComputePID(float  input, float Target_Anglex)
{

    Angle_Error = Target_Anglex-input;
    Proportional = Proportional_Factor * Angle_Error;
    Integral +=  Angle_Error * dt;
    if(  Integral>4)
    {
    	Integral=4;
    }
if(Integral<-4)
{
	Integral=-4;
}
else
{
	Integral=0;
}
    /* if( -5< Angle_Error && Angle_Error < 5)
    {
    	Integral +=  Angle_Error * dt;
    	flag=1;
    }
    else
    {
    	flag=0;
    }*/
    Derivative =  (Angle_Error - Previous_Error) ;
    if( Derivative>20)
    {
    	Derivative=20;
    }
    if(Derivative<-20)
    {
    	Derivative=-20;
    }
   output = Proportional + Integral*Integral_Factor+ Derivative*Derivative_Factor;

    Previous_Error = Angle_Error;
    return output;
}
void motor()
{

	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,200);
	  		   	 	  HAL_Delay(4000);
	  		   	 __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);
	  		   	       HAL_Delay(4000);


}
void motormin()
{
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);
}
void motormax()
{
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,180);


}
     void PWM_SetCompare1(uint16_t Compar)
     {
    	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

     	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Compar);

     }
     void PWM_SetCompare2(uint16_t Compare)
         {
        	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

         	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,Compare);

         }

     void Servo_SetAngle(float Angl,float Ang)
     {
     	PWM_SetCompare1(Ang/180*200+50);
     	PWM_SetCompare2(Angl/180*200+50);
     	HAL_Delay (220);
     }
     void PitchServo_SetAnglex(float Angl)
         {
         	PWM_SetCompare1(Angl/180*200+50);
         	PWM_SetCompare2(Angl/180*200+50);
         	HAL_Delay (220);
         }
     void RollServo_SetRoll(float Rol,float Ro)
         {
         	PWM_SetCompare2(Ro/180*200+50);
         	PWM_SetCompare1(Rol/180*200+50);
         	HAL_Delay (220);

         }

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
