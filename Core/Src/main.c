/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>
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
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RxBuf_SIZE   5
#define MainBuf_SIZE 5

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];

uint16_t oldPos = 0;
uint16_t newPos = 0;

#define MAX_LED 24
#define USE_BRIGHTNESS 1
#define PI 3.14159265
int datasentflag=0;
uint16_t effStep = 1;
uint8_t LED_Data[MAX_LED][8];
uint8_t LED_Mod[MAX_LED][8];  // for brightness

char value[] = {'0','0','0','0'};
char color[3];


uint16_t pwmData[(24*MAX_LED)+50];

uint16_t Rx_val = 0;
uint16_t val = 0;
int eff = 0;

uint16_t RxBufint = 0;

uint8_t Brightness;



void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

static void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS
	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif
}



static void WS2812_Send (void)
	{
		uint32_t indx=0;
		uint32_t color;
		for (int i= 0; i<MAX_LED; i++)
		{
	#if USE_BRIGHTNESS
			color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
	#else
			color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
	#endif

			for (int i=23; i>=0; i--)
			{
				if (color&(1<<i))
				{
					pwmData[indx] = 60;  // 2/3 of 90
				}
				else pwmData[indx] = 30;  // 1/3 of 90
				indx++;
			}
		}
		for (int i=0; i<50; i++)
		{
			pwmData[indx] = 0;
			indx++;
		}

		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
		while (!datasentflag){};
		datasentflag = 0;
	}

void Reset_LED (void)
	{
		for (int i=0; i<MAX_LED; i++)
		{
			LED_Data[i][0] = i;
			LED_Data[i][1] = 0;
			LED_Data[i][2] = 0;
			LED_Data[i][3] = 0;
		}
	}

static uint8_t rainbow_effect_left() {

	  float factor1, factor2;
	  uint16_t ind;
	  for(uint16_t j=0;j<24;j++) {
	    ind = effStep + j * 1.625;
	    switch((int)((ind % 13) / 4.333333333333333)) {
	      case 0: factor1 = 1.0 - ((float)(ind % 13 - 0 * 4.333333333333333) / 4.333333333333333);
	              factor2 = (float)((int)(ind - 0) % 13) / 4.333333333333333;
	              /************ chnaged here *********/
	              Set_LED(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
	              WS2812_Send();
	              break;
	      case 1: factor1 = 1.0 - ((float)(ind % 13 - 1 * 4.333333333333333) / 4.333333333333333);
	              factor2 = (float)((int)(ind - 4.333333333333333) % 13) / 4.333333333333333;
	              Set_LED(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
	              WS2812_Send();
	              break;
	      case 2: factor1 = 1.0 - ((float)(ind % 13 - 2 * 4.333333333333333) / 4.333333333333333);
	              factor2 = (float)((int)(ind - 8.666666666666666) % 13) / 4.333333333333333;
	              Set_LED(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
	              WS2812_Send();
	              break;
	    }
	  }
	  if(effStep >= 13) {effStep=0; return 0x03; }
	  else effStep++;
	  return 0x01;
	}

static uint8_t rainbow_effect_right() {
	    // Strip ID: 0 - Effect: Rainbow - LEDS: 8
	    // Steps: 14 - Delay: 30
	    // Colors: 3 (255.0.0, 0.255.0, 0.0.255)
	    // Options: rainbowlen=8, toLeft=false,
	//  if(millis() - strip_0.effStart < 30 * (strip_0.effStep)) return 0x00;
	  float factor1, factor2;
	  uint16_t ind;
	  for(uint16_t j=0;j<24;j++) {
	    ind = 14 - (int16_t)(effStep - j * 1.75) % 14;
	    switch((int)((ind % 14) / 4.666666666666667)) {
	      case 0: factor1 = 1.0 - ((float)(ind % 14 - 0 * 4.666666666666667) / 4.666666666666667);
	              factor2 = (float)((int)(ind - 0) % 14) / 4.666666666666667;
	              Set_LED(j, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2);
	              WS2812_Send();
	              break;
	    	  factor1 = 1.0 - ((float)(ind % 14 - 2 * 4.666666666666667) / 4.666666666666667);
	    	  	              factor2 = (float)((int)(ind - 9.333333333333334) % 14) / 4.666666666666667;
	    	  	              Set_LED(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
	    	  	              WS2812_Send();
	    	  	              break;

	      case 1: factor1 = 1.0 - ((float)(ind % 14 - 1 * 4.666666666666667) / 4.666666666666667);
	              factor2 = (float)((int)(ind - 4.666666666666667) % 14) / 4.666666666666667;
	              Set_LED(j, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2, 0 * factor1 + 255 * factor2);
	              WS2812_Send();
	              break;
	      case 2: factor1 = 1.0 - ((float)(ind % 14 - 2 * 4.666666666666667) / 4.666666666666667);
	              factor2 = (float)((int)(ind - 9.333333333333334) % 14) / 4.666666666666667;
	              Set_LED(j, 0 * factor1 + 255 * factor2, 0 * factor1 + 0 * factor2, 255 * factor1 + 0 * factor2);
	              WS2812_Send();
	              break;
	    }
	  }
	  if(effStep >= 14) {effStep = 0; return 0x03; }
	  else effStep++;
	  return 0x01;
	}

static void white_effect (void){
		for(uint8_t j=0;j<24;j++) {
		  Set_LED(j, 255, 255, 255);
		  WS2812_Send();
		}
	}
static void color_effect(uint8_t *RxBuf){

	color[0] = RxBuf[2];
	color[1] = RxBuf[3];
	color[2] = RxBuf[4];

	for(uint8_t j=0;j<24;j++) {
			  Set_LED(j, color[0], color[1], color[2]);
			  WS2812_Send();
			}
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
		Rx_val = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		//copy RxBuf to MainBuf (array type)
		memcpy (MainBuf, RxBuf,MainBuf_SIZE+1);
		memset(RxBuf,0,1);//Delete RxBuf

	}
	// converting string to integer
	for (int i = 1; MainBuf[i] != '\0'; i++)
	{
		Rx_val = Rx_val * 10 + (MainBuf[i] - 48);
	}


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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
     __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
     while (1)
     {
    	 /* USER CODE END WHILE */

    	 /* USER CODE BEGIN 3 */


    	 switch(*MainBuf){
    	 case 'A':
    		 rainbow_effect_left();
    		 Brightness = RxBuf[1];
    		 Set_Brightness(Brightness);
    		 break;

    	 case 'B':
    		 rainbow_effect_right();
    		 Brightness = RxBuf[1];
    		 Set_Brightness(Brightness);
    		 break;

    	 case 'D':
    	     color_effect(RxBuf);
    	     Brightness = RxBuf[1];
    	     Set_Brightness(Brightness);
    	     break;
    	 }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 90-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
