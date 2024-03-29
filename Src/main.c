/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
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
float value,value2,dioda;
int value1,value3=0,value4,error;
char buffer[40];
int PWM,Wzadana=2000;
uint8_t size,Received,dane[6],daneint[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart3, &Received, 1);
	for(int i =0; i==0; i++){
	if (Received=='&' && dane[0]==0) {
		dane[0] = '&';
		break;
	}


	 if (Received>47 && Received<58 && dane[0] =='&' && dane[1]==0) {
		dane[1] = Received;
		break;
	}


	else if (Received>47 && Received<58 && dane[0] =='&' && dane[1]!=0 && dane[2] ==0) {
				dane[2] = Received;
				break;
			}


	else if ( Received>47 && Received<58 && dane[0] =='&' && dane[1]!=0 && dane[2] !=0 && dane[3]==0) {
					dane[3] = Received;
					break;
				}


	else if (Received>47 && Received<58 && dane[0] =='&' && dane[1]!=0 && dane[2] !=0 && dane[3]!=0 && dane[4]==0) {
					dane[4] = Received;
					break;
				}


	else if ( Received =='*' && dane[0] =='&' && dane[1]!=0 && dane[2] !=0 && dane[3]!=0 && dane[4]!=0 )
							{
					for(int i =0; i<=3; i++){
						daneint[i] = dane[i+1];
						Wzadana = atoi(&daneint);
					}
					for(int i =0; i<=5; i++){
						dane[i] =0;
					}
					break;
				}
		else {
					for(int i =0; i<=5; i++){
											dane[i] = 0;
										}}

	}
}



// HAL_UART_Receive_IT(&huart3, &Received, 4); // Ponowne włączenie nasłuchiwania

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  arm_pid_instance_f32 PID;
  PID.Kp = 0.04;
  PID.Ki = 0.009;
  PID.Kd = 0.0;
  arm_pid_init_f32(&PID,1);
  HAL_UART_Receive_IT(&huart3, &Received, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  for(int i=0; i<100;i++){
		  HAL_ADC_Start(&hadc1);
		  if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
			  value = HAL_ADC_GetValue(&hadc1);
			  value2 = value*0.00080566;
			  value1 = (value2*4630.0)/(3.3-value2);
			  value3 +=value1;
		  }
	  }
	  value4 = value3/100;
	  value3 =0;
	  size = sprintf(buffer, "Value: %d [Ohm]\n\r", value4);
	  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 200);
	  HAL_Delay(20);

	  error = value4-Wzadana;
	  dioda =  (abs(error)*1.000) /Wzadana*1.00;
	  if(dioda <= 0.01){
		  HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
	  }
	  else if(dioda >0.01 && dioda <=0.05){
		  	  	  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
	  }
	  else if( dioda >0.05){
			  	  	  HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, GPIO_PIN_SET);
					  HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, GPIO_PIN_RESET);
		  }
	  PWM = arm_pid_f32(&PID,error);
	  if (PWM > 1000) {
	                  PWM = 1000;
	                 // PID.state[2] = 100;

	              }
	  else if (PWM < 60) {
	                  PWM = 60;
	                //  PID.state[2] = 0;
	              }
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM);


	}




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc)
{
	//value = HAL_ADC_GetValue(hadc);
	//HAL_ADC_Stop_IT(hadc);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
