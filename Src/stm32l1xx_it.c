/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l1xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "mbport.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HardFault_Handler_C( unsigned int *args )
{
    volatile unsigned int stacked_r0;
    volatile unsigned int stacked_r1;
    volatile unsigned int stacked_r2;
    volatile unsigned int stacked_r3;
    volatile unsigned int stacked_r12;
    volatile unsigned int stacked_lr;
    volatile unsigned int stacked_pc;
    volatile unsigned int stacked_psr;

    stacked_r0 = ( ( unsigned long) args[0] );
    stacked_r1 = ( ( unsigned long) args[1] );
    stacked_r2 = ( ( unsigned long) args[2] );
    stacked_r3 = ( ( unsigned long) args[3] );

    stacked_r12 = ( ( unsigned long) args[4] );
    stacked_lr = ( ( unsigned long) args[5] );
    stacked_pc = ( ( unsigned long) args[6] );
    stacked_psr = ( ( unsigned long) args[7] );

    ( void )stacked_r0;
    ( void )stacked_r1;
    ( void )stacked_r2;
    ( void )stacked_r3;

    ( void )stacked_r12;
    ( void )stacked_lr ;
    ( void )stacked_pc ;
    ( void )stacked_psr;

    // WWDG start
//    Wdt_t wwdt;
//    WdtInit( &wwdt, WDT_WWDG);

    while( 1 );
}

#if defined(__CC_ARM)
__asm void HardFault_Handler(void)
{
    TST LR, #4
    ITE EQ
    MRSEQ r0, MSP
    MRSNE r0, PSP
    B __cpp(HardFault_Handler_C)
}
#elif defined(__ICCARM__)
void HardFault_Handler(void)
{
    __asm("TST LR, #4");
    __asm("ITE EQ");
    __asm("MRSEQ r0, MSP");
    __asm("MRSNE r0, PSP");
    __asm("B HardFault_Handler_C");
}
#elif defined(__GNUC__)
void HardFault_Handler(void)
{
    __asm volatile( "TST LR, #4" );
    __asm volatile( "ITE EQ" );
    __asm volatile( "MRSEQ R0, MSP" );
    __asm volatile( "MRSNE R0, PSP" );
    __asm volatile( "B HardFault_Handler_C" );
}
#else
    #warning Not supported compiler type
#endif
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
//void HardFault_Handler(void)
//{
//  /* USER CODE BEGIN HardFault_IRQn 0 */
//
//  /* USER CODE END HardFault_IRQn 0 */
//  while (1)
//  {
//    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
//    /* USER CODE END W1_HardFault_IRQn 0 */
//  }
//}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USB low priority interrupt.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	uint32_t tmp_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
	uint32_t tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE);

	if ((tmp_flag != RESET) && (tmp_it_source != RESET)) {
		pxMBMasterFrameCBByteReceived();
		__HAL_UART_CLEAR_PEFLAG(&huart1);
		return;
	}

	if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET)
			&& (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) != RESET)) {
		pxMBMasterFrameCBTransmitterEmpty();
		return;
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void RTC_Alarm_IRQHandler( void )
{
  HW_RTC_IrqHandler ( );
}

void EXTI0_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_0 );
}

void EXTI1_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
}

void EXTI2_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );
}

void EXTI3_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
}

void EXTI4_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
}

void EXTI9_5_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
}

void EXTI15_10_IRQHandler( void )
{

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );

  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );


}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
