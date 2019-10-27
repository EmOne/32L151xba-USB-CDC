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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#ifdef __CROSS_STUDIO__
#include <__cross_studio_io.h>
#endif
#include "mbconfig.h"
//#include "mbport.h"
#include "util_console.h"
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
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

IWDG_HandleTypeDef hiwdg;

//WWDG_HandleTypeDef hwwdg;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
osThreadId simulateTaskHandle;
osThreadId modbusMasterTaskHandle;
#elif MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0
osThreadId modbusSlaveTaskHandle;
#else
#error	"Please define MB_MASTER_RTU_ENABLED or MB_SLAVE_RTU_ENABLED"
#endif

#ifdef USE_LORA_HAL_DRIVER
osThreadId mLoraTaskHandle;
extern void thread_LoRaPoll(void const * argument);
#endif

uint16_t CpuUsageMajor;
uint16_t CpuUsageMinor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
//static void MX_RTC_Init(void);
//static void MX_SPI1_Init(void);
//static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
//static void MX_WWDG_Init(void);
//void StartDefaultTask(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
extern void thread_Simulation(void const * argument);
extern void thread_ModbusMasterPoll(void const * argument);
extern void prvvMastesTIMERExpiredISR(void);
#else
extern void thread_ModbusSlavePoll(void const * argument);
extern void prvvTIMERExpiredISR(void);
#endif

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
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_IWDG_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();
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
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
	osThreadDef(simulateTask, thread_Simulation, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE * 4);
	simulateTaskHandle = osThreadCreate(osThread(simulateTask), NULL);

	osThreadDef(mModbusTask, thread_ModbusMasterPoll, osPriorityNormal, 0,
			configMINIMAL_STACK_SIZE  * 4);
	modbusMasterTaskHandle = osThreadCreate(osThread(mModbusTask), NULL);
#else
	  osThreadDef(sModbusTask, thread_ModbusSlavePoll, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
	  modbusSlaveTaskHandle = osThreadCreate(osThread(sModbusTask),  NULL);
#endif
#ifdef USE_LORA_HAL_DRIVER
	  osThreadDef(mLoRaTask, thread_LoRaPoll, osPriorityNormal, 0,
	  			configMINIMAL_STACK_SIZE  * 6);
	  mLoraTaskHandle = osThreadCreate(osThread(mLoRaTask), NULL);
#endif
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC_Init(void)
//{
//
//  /* USER CODE BEGIN ADC_Init 0 */
//
//  /* USER CODE END ADC_Init 0 */
//
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC_Init 1 */
//
//  /* USER CODE END ADC_Init 1 */
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//  hadc.Instance = ADC1;
//  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//  hadc.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
//  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
//  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
//  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
//  hadc.Init.ContinuousConvMode = DISABLE;
//  hadc.Init.NbrOfConversion = 1;
//  hadc.Init.DiscontinuousConvMode = DISABLE;
//  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc.Init.DMAContinuousRequests = DISABLE;
//  if (HAL_ADC_Init(&hadc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC_Init 2 */
//
//  /* USER CODE END ADC_Init 2 */
//
//}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_RTC_Init(void)
//{
//
//  /* USER CODE BEGIN RTC_Init 0 */
//
//  /* USER CODE END RTC_Init 0 */
//
//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef sDate = {0};
//
//  /* USER CODE BEGIN RTC_Init 1 */
//
//  /* USER CODE END RTC_Init 1 */
//  /** Initialize RTC Only
//  */
//  hrtc.Instance = RTC;
//  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
//  hrtc.Init.AsynchPrediv = 127;
//  hrtc.Init.SynchPrediv = 255;
//  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
//  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
//  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
//  if (HAL_RTC_Init(&hrtc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* USER CODE BEGIN Check_RTC_BKUP */
//
//  /* USER CODE END Check_RTC_BKUP */
//
//  /** Initialize RTC and set the Time and Date
//  */
//  sTime.Hours = 0x0;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
//  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
//  sDate.Month = RTC_MONTH_JANUARY;
//  sDate.Date = 0x1;
//  sDate.Year = 0x0;
//
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN RTC_Init 2 */
//
//  /* USER CODE END RTC_Init 2 */
//
//}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SPI1_Init(void)
//{
//
//  /* USER CODE BEGIN SPI1_Init 0 */
//
//  /* USER CODE END SPI1_Init 0 */
//
//  /* USER CODE BEGIN SPI1_Init 1 */
//
//  /* USER CODE END SPI1_Init 1 */
//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI1_Init 2 */
//
//  /* USER CODE END SPI1_Init 2 */
//
//}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{
	/*##-2- Init & Start WWDG peripheral ######################################*/
	  /* WWDG clock counter = (PCLK1 (32MHz)/4096)/8) = 976 Hz (~1024 us)
	     WWDG Window value = 80 means that the WWDG counter should be refreshed only
	     when the counter is below 80 (and greater than 64/0x40) otherwise a reset will
	     be generated.
	     WWDG Counter value = 127, WWDG timeout = ~1024 us * 64 = 65.57 ms
	     In this case the refresh window is comprised between : ~1024 * (127-80) = 48.1ms and ~1024 * 64 = 65.53ms
	   */
		hiwdg.Instance = IWDG;
		hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
		hiwdg.Init.Reload    = 0xFFF;

	  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	  {
	    /* Initialization Error */
	    Error_Handler();
	  }

}

///**
//  * @brief WWDG Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_WWDG_Init(void) {
//	/*##-2- Init & Start WWDG peripheral ######################################*/
//	/* WWDG clock counter = (PCLK1 (32MHz)/4096)/8) = 976 Hz (~1024 us)
//	 WWDG Window value = 80 means that the WWDG counter should be refreshed only
//	 when the counter is below 80 (and greater than 64/0x40) otherwise a reset will
//	 be generated.
//	 WWDG Counter value = 127, WWDG timeout = ~1024 us * 64 = 65.57 ms
//	 In this case the refresh window is comprised between : ~1024 * (127-80) = 48.1ms and ~1024 * 64 = 65.53ms
//	 */
//
//	hwwdg.Instance = WWDG;
//	hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
//	hwwdg.Init.Window = 0x7F;
//	hwwdg.Init.Counter = 0x7F;
//	hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
//
//	if (HAL_WWDG_Init(&hwwdg) != HAL_OK) {
//		/* Initialization Error */
//		Error_Handler();
//	}
//
//}


/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  HAL_GPIO_WritePin(RX_TX_GPIO_Port, RX_TX_Pin, GPIO_PIN_SET);
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 4800;
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
  HAL_GPIO_WritePin(RX_TX_GPIO_Port, RX_TX_Pin, GPIO_PIN_RESET);
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RX_TX_Pin|RADIO_RESET_Pin|RADIO_ANT_SWITCH_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RADIO_ANT_SWITCH_RX_Pin */
  GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_ANT_SWITCH_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RX_TX_Pin */
  GPIO_InitStruct.Pin = RX_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(RX_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_RESET_Pin RADIO_ANT_SWITCH_TX_Pin */
  GPIO_InitStruct.Pin = RADIO_RESET_Pin|RADIO_ANT_SWITCH_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_NSS_Pin */
  GPIO_InitStruct.Pin = RADIO_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADIO_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_DIO_0_Pin RADIO_DIO_1_Pin RADIO_DIO_2_Pin RADIO_DIO_5_Pin
                           RADIO_DIO_4_Pin RADIO_DIO_3_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_0_Pin|RADIO_DIO_1_Pin|RADIO_DIO_2_Pin|RADIO_DIO_5_Pin
                          |RADIO_DIO_4_Pin|RADIO_DIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14
                           PB15 PB6 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
//void StartDefaultTask(void const * argument)
//{
//  /* init code for USB_DEVICE */
////  MX_USB_DEVICE_Init();
//
//  /* USER CODE BEGIN 5 */
//
//  /* Infinite loop */
//  for(;;)
//  {
//	  osDelay(1);
//  }
//  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM7) {

	  (void) prvvMastesTIMERExpiredISR();
  }

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
	  PRINTF("Error_Handler\n\r");
	  while (1)
	  {
	    ;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
