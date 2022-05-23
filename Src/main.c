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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "hd44780.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId ButtonsTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartButtonsTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern const unsigned short int DACLookup_FullSine_7Bit[128];
unsigned char Sine1_Out[256], Sine2_Out[256], I2C_Send_Buf[2], UpDown = 0, ADC_nbr = 0;
char str[16];
unsigned short int DAC1_pos = 0, DAC2_pos = 0;
float Amplitude1 = 0, Amplitude2 = 0;
unsigned char cnt20ms = 0;
bool LCD_refresh = true, DAC_refresh = true;

const uint16_t cb_rep_spd_tab[4] = {15, 7, 4, 1};
unsigned char kbrd_code = 0;
unsigned char cb_rep_spd = 0;
unsigned short int cb_rep = 0, cb_rep_cnt = 0, cb1 = 0, cb2 = 0, cb3 = 0, cb4 = 0, cb5 = 0;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	__nop();
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ButtonsTask */
  osThreadDef(ButtonsTask, StartButtonsTask, osPriorityNormal, 0, 128);
  ButtonsTaskHandle = osThreadCreate(osThread(ButtonsTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 1000000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 77;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TEST_LED_Pin|TEST_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_E_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin 
                          |LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEST_LED_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TEST_OUT_Pin */
  GPIO_InitStruct.Pin = TEST_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TEST_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_E_Pin LCD_RS_Pin LCD_D4_Pin LCD_D5_Pin 
                           LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_E_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin 
                          |LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
/*
//	HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0, I2C_MEMADD_SIZE_16BIT, I2C_Read_Buf, 32, 100);
	HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_SET);
	I2C_Send_Buf[0] = 0;
	I2C_Send_Buf[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, 0xA0, I2C_Send_Buf, 2, 100);
	HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
*/
  /* Infinite loop */
//	HAL_GPIO_WritePin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim3);
  for(;;)
  {
//    sprintf(str, "%04d",adc);
//    lcdGoto(LCD_1st_LINE,1);
//    lcdPuts(str);
//		HAL_I2C_Master_Transmit(&hi2c1, 0xC4, Sine_Out, 1024, 100);
//		HAL_I2C_Master_Transmit_DMA(&hi2c1, 0xC4, Sine_Out, 1024);
/*		I2C_Send_Buf[0] = 64;
		I2C_Send_Buf[1] = (unsigned char)(FullSine_9Bit[DAC_pos] / 16);
		I2C_Send_Buf[2] = (unsigned char)((FullSine_9Bit[DAC_pos] % 16) << 4);
		HAL_I2C_Master_Transmit_IT(&hi2c1,  0xC4, I2C_Send_Buf, 3);
*/
/*
		for(i = 0; i < 128; i++)
		{
			tmp = DACLookup_FullSine_7Bit[i] * Amplitude1;
			Sine1_Out[i*2] = (unsigned char)(tmp >> 8);
			Sine1_Out[(i*2)+1] = (unsigned char)tmp;
			tmp = DACLookup_FullSine_7Bit[i] * Amplitude2;
			Sine2_Out[i*2] = (unsigned char)(tmp >> 8);
			Sine2_Out[(i*2)+1] = (unsigned char)tmp;
		}
		osDelay(100);
		if(UpDown == 0)
		{
			Amplitude1 = Amplitude1 - 0.01;
			Amplitude2 = Amplitude2 - 0.01;
			if(Amplitude1 < 0.5)
			{
				UpDown = 1;
			}
		}
		else
		{
			Amplitude1 = Amplitude1 + 0.01;
			Amplitude2 = Amplitude2 + 0.01;
			if(Amplitude1 > 1)
			{
				Amplitude1 = 1;
				Amplitude2 = 1;
				UpDown = 0;
			}
		}
*/
osDelay(200);
//TM1638_Send();
/*
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[0] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[1] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[2] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[3] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[4] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[5] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[6] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[7] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[8] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[9] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[10] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[11] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[12] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[13] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[14] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[15] = 0xFF;
TM1638_Buttons();
osDelay(1000);
TM1638_LED_Buf[0] = 0x00;TM1638_LED_Buf[1] = 0x00;TM1638_LED_Buf[2] = 0x00;TM1638_LED_Buf[3] = 0x00;TM1638_LED_Buf[4] = 0x00;
TM1638_LED_Buf[5] = 0x00;TM1638_LED_Buf[6] = 0x00;TM1638_LED_Buf[7] = 0x00;TM1638_LED_Buf[8] = 0x00;TM1638_LED_Buf[9] = 0x00;
TM1638_LED_Buf[10] = 0x00;TM1638_LED_Buf[11] = 0x00;TM1638_LED_Buf[12] = 0x00;TM1638_LED_Buf[13] = 0x00;TM1638_LED_Buf[14] = 0x00;
TM1638_LED_Buf[15] = 0x00;
*/
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartButtonsTask */
/**
* @brief Function implementing the ButtonsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonsTask */
void StartButtonsTask(void const * argument)
{
  /* USER CODE BEGIN StartButtonsTask */
	unsigned short int adc, tmp;
	unsigned char key, i;

	lcdInit();
	lcdClrScr();
  lcdGoto(LCD_1st_LINE,1);
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100);
    adc = (unsigned short int)HAL_ADC_GetValue(&hadc1);
		if(adc > 3400)
		{
			key = 0;
		}
		else
		{
			if(adc > 2300)
			{
				key = 1;
			}
			else
			{
				if(adc > 1435)
				{
					key = 2;
				}
				else
				{
					if(adc > 690)
					{
						key = 3;
					}
					else
					{
						if(adc > 157)
						{
							key = 4;
						}
						else
						{
							key = 5;
						}
					}
				}
			}
		}
		if(key == 1)
		{
			if(cb1 != 3)
			{
				cb1++;
				if(cb1 == 3)
				{
					kbrd_code = 1;
					cb_rep = 0;
					cb_rep_spd = 0;
					cb_rep_cnt = 0;
				}
			}
			else
			{
				cb_rep++;
				if(cb_rep >= cb_rep_spd_tab[cb_rep_spd])
				{
					kbrd_code = 1;
					cb_rep = 0;
				}
			}
		}
		else
		{
			cb1 = 0;
		}
		if(key == 2)
		{
			if(cb2 != 3)
			{
				cb2++;
				if(cb2 == 3)
				{
					kbrd_code = 2;
					cb_rep = 0;
					cb_rep_spd = 0;
					cb_rep_cnt = 0;
				}
			}
			else
			{
				cb_rep++;
				if(cb_rep >= cb_rep_spd_tab[cb_rep_spd])
				{
					kbrd_code = 2;
					cb_rep = 0;
				}
			}
		}
		else
		{
			cb2 = 0;
		}
		if(key == 3)
		{
			if(cb3 != 3)
			{
				cb3++;
				if(cb3 == 3)
				{
					kbrd_code = 3;
					cb_rep = 0;
					cb_rep_spd = 0;
					cb_rep_cnt = 0;
				}
			}
			else
			{
				cb_rep++;
				if(cb_rep >= cb_rep_spd_tab[cb_rep_spd])
				{
					kbrd_code = 3;
					cb_rep = 0;
				}
			}
		}
		else
		{
			cb3 = 0;
		}
		if(key == 4)
		{
			if(cb4 != 3)
			{
				cb4++;
				if(cb4 == 3)
				{
					kbrd_code = 4;
					cb_rep = 0;
					cb_rep_spd = 0;
					cb_rep_cnt = 0;
				}
			}
			else
			{
				cb_rep++;
				if(cb_rep >= cb_rep_spd_tab[cb_rep_spd])
				{
					kbrd_code = 4;
					cb_rep = 0;
				}
			}
		}
		else
		{
			cb4 = 0;
		}
		if(key == 5)
		{
			if(cb5 != 3)
			{
				cb5++;
				if(cb5 == 3)
				{
					kbrd_code = 5;
					cb_rep = 0;
					cb_rep_spd = 0;
					cb_rep_cnt = 0;
				}
			}
			else
			{
				cb_rep++;
				if(cb_rep >= cb_rep_spd_tab[cb_rep_spd])
				{
					kbrd_code = 5;
					cb_rep = 0;
				}
			}
		}
		else
		{
			cb5 = 0;
		}
		if(cb_rep != 0)
		{
			if(cb_rep_cnt != 200)
			{
				cb_rep_cnt++;
				if(cb_rep_cnt == 50)
				{
					cb_rep_spd = 1;
				}
				if(cb_rep_cnt == 100)
				{
					cb_rep_spd = 2;
				}
				if(cb_rep_cnt == 200)
				{
					cb_rep_spd = 3;
				}
			}
		}
		switch(kbrd_code)
		{
			case 1:
			{
				kbrd_code = 0;
				break;
			}
			case 2:
			{
				kbrd_code = 0;
				if((Amplitude1 - 0.001) > 0)
				{
					Amplitude1 = Amplitude1 - 0.001;
					LCD_refresh = true;
					DAC_refresh = true;
				}
				else
				{
					Amplitude1 = 0;
					LCD_refresh = true;
					DAC_refresh = true;
				}
				break;
			}
			case 3:
			{
				kbrd_code = 0;
				if((Amplitude2 - 0.0015) > 0)
				{
					Amplitude2 = Amplitude2 - 0.0015;
					LCD_refresh = true;
					DAC_refresh = true;
				}
				else
				{
					Amplitude2 = 0;
					LCD_refresh = true;
					DAC_refresh = true;
				}
				break;
			}
			case 4:
			{
				kbrd_code = 0;
				if(Amplitude2 < 0.45)
				{
					Amplitude2 = Amplitude2 + 0.0015;
					LCD_refresh = true;
					DAC_refresh = true;
				}
				break;
			}
			case 5:
			{
				kbrd_code = 0;
				if(Amplitude1 < 0.03)
				{
					Amplitude1 = Amplitude1 + 0.001;
					LCD_refresh = true;
					DAC_refresh = true;
				}
				break;
			}
		}
		if(LCD_refresh)
		{
			LCD_refresh = false;
			sprintf(str, "%03u", (unsigned short int)(Amplitude1 * 1000));
			lcdGoto(LCD_1st_LINE,1);
			lcdPuts(str);
			sprintf(str, "%03u", (unsigned short int)(Amplitude2 * 666.5));
			lcdGoto(LCD_2nd_LINE,1);
			lcdPuts(str);
		}
		if(DAC_refresh)
		{
			DAC_refresh = false;
			for(i = 0; i < 128; i++)
			{
				tmp = DACLookup_FullSine_7Bit[i] * Amplitude1;
				Sine1_Out[i*2] = (unsigned char)(tmp >> 8);
				Sine1_Out[(i*2)+1] = (unsigned char)tmp;
				tmp = DACLookup_FullSine_7Bit[i] * Amplitude2;
				Sine2_Out[i*2] = (unsigned char)(tmp >> 8);
				Sine2_Out[(i*2)+1] = (unsigned char)tmp;
			}
		}
		osDelay(20);
  }
  /* USER CODE END StartButtonsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if(htim->Instance == TIM3)
	{
		HAL_GPIO_TogglePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin);
		if(ADC_nbr == 0)
		{
			I2C_Send_Buf[0] = Sine1_Out[DAC1_pos];
			DAC1_pos++;
			I2C_Send_Buf[1] = Sine1_Out[DAC1_pos];
			DAC1_pos++;
			HAL_I2C_Master_Transmit_DMA(&hi2c1,  0xC2, I2C_Send_Buf, 2);
			if(DAC1_pos == 256)
			{
				DAC1_pos = 0;
			}
			ADC_nbr = 1;
		}
		else
		{
			I2C_Send_Buf[0] = Sine2_Out[DAC2_pos];
			DAC2_pos++;
			I2C_Send_Buf[1] = Sine2_Out[DAC2_pos];
			DAC2_pos++;
			HAL_I2C_Master_Transmit_DMA(&hi2c1,  0xC4, I2C_Send_Buf, 2);
			if(DAC2_pos == 256)
			{
				DAC2_pos = 0;
			}
			ADC_nbr = 0;
		}
	}
  if(htim->Instance == TIM4)
	{
		cnt20ms++;
		if(cnt20ms == 20)
		{
			cnt20ms = 0;
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
