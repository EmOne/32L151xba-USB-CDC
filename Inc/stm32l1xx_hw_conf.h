/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: contains hardaware configuration Macros and Constants

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    stm32l1xx_hw_conf.h
  * @author  MCD Application Team
  * @brief   contains hardaware configuration Macros and Constants for stm32l1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONF_L1_H__
#define __HW_CONF_L1_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

//#define RADIO_DIO_4
//#define RADIO_DIO_5

/* LORA I/O definition */

#ifdef USE_SX126X_DVK

#define RADIO_RESET_PORT                          GPIOA
#define RADIO_RESET_PIN                           GPIO_PIN_0

#define RADIO_MOSI_PORT                           GPIOA
#define RADIO_MOSI_PIN                            GPIO_PIN_7

#define RADIO_MISO_PORT                           GPIOA
#define RADIO_MISO_PIN                            GPIO_PIN_6

#define RADIO_SCLK_PORT                           GPIOA
#define RADIO_SCLK_PIN                            GPIO_PIN_5

#define RADIO_NSS_PORT                            GPIOA
#define RADIO_NSS_PIN                             GPIO_PIN_8

#define RADIO_BUSY_PORT                           GPIOB
#define RADIO_BUSY_PIN                            GPIO_PIN_3

#define RADIO_DIO_0_PORT                          GPIOA
#define RADIO_DIO_0_PIN                           GPIO_PIN_10

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_4

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_5

#define RADIO_DIO_3_PORT                          GPIOB
#define RADIO_DIO_3_PIN                           GPIO_PIN_4

#define RADIO_ANT_SWITCH_POWER_PORT               GPIOA
#define RADIO_ANT_SWITCH_POWER_PIN                GPIO_PIN_9

#define DEVICE_SEL_PORT                           GPIOA
#define DEVICE_SEL_PIN                            GPIO_PIN_4

#define RADIO_LEDTX_PORT                           GPIOC
#define RADIO_LEDTX_PIN                            GPIO_PIN_1

#define RADIO_LEDRX_PORT                           GPIOC
#define RADIO_LEDRX_PIN                            GPIO_PIN_0

#elif defined ( USE_IM980A )

#define RADIO_RESET_PORT                          RADIO_RESET_GPIO_Port
#define RADIO_RESET_PIN                           RADIO_RESET_Pin

#define RADIO_MISO_PIN 							RADIO_MISO_Pin
#define RADIO_MISO_PORT 						RADIO_MISO_GPIO_Port

#define RADIO_MOSI_PIN 							RADIO_MOSI_Pin
#define RADIO_MOSI_PORT 						RADIO_MOSI_GPIO_Port

#define RADIO_NSS_PIN 							RADIO_NSS_Pin
#define RADIO_NSS_PORT 							RADIO_NSS_GPIO_Port

#define RADIO_SCLK_PIN 							RADIO_SCK_Pin
#define RADIO_SCLK_PORT 						RADIO_SCK_GPIO_Port

#define RADIO_DIO_0_PORT                          RADIO_DIO_0_GPIO_Port
#define RADIO_DIO_0_PIN                           RADIO_DIO_0_Pin

#define RADIO_DIO_1_PORT                          RADIO_DIO_1_GPIO_Port
#define RADIO_DIO_1_PIN                           RADIO_DIO_1_Pin

#define RADIO_DIO_2_PORT                          RADIO_DIO_2_GPIO_Port
#define RADIO_DIO_2_PIN                           RADIO_DIO_2_Pin

#define RADIO_DIO_3_PORT                          RADIO_DIO_3_GPIO_Port
#define RADIO_DIO_3_PIN                           RADIO_DIO_3_Pin

#ifdef RADIO_DIO_4
#define RADIO_DIO_4_PORT                          RADIO_DIO_4_GPIO_Port
#define RADIO_DIO_4_PIN                           RADIO_DIO_4_Pin
#endif

#ifdef RADIO_DIO_5
#define RADIO_DIO_5_PORT                          RADIO_DIO_5_GPIO_Port
#define RADIO_DIO_5_PIN                           RADIO_DIO_5_Pin
#endif

//#define RADIO_ANT_SWITCH_PORT                     RADIO_ANT_SWITCH_RX_GPIO_Port
//#define RADIO_ANT_SWITCH_PIN                      RADIO_ANT_SWITCH_RX_Pin

#define RADIO_ANT_SWITCH_RX_Pin 				GPIO_PIN_13
#define RADIO_ANT_SWITCH_RX_GPIO_Port 			GPIOC

#define RADIO_ANT_SWITCH_TX_Pin 				GPIO_PIN_4
#define RADIO_ANT_SWITCH_TX_GPIO_Port 			GPIOA

#else


#define RADIO_RESET_PORT                          GPIOA
#define RADIO_RESET_PIN                           GPIO_PIN_0

#define RADIO_MOSI_PORT                           GPIOA
#define RADIO_MOSI_PIN                            GPIO_PIN_7

#define RADIO_MISO_PORT                           GPIOA
#define RADIO_MISO_PIN                            GPIO_PIN_6

#define RADIO_SCLK_PORT                           GPIOA
#define RADIO_SCLK_PIN                            GPIO_PIN_5

#define RADIO_NSS_PORT                            GPIOB
#define RADIO_NSS_PIN                             GPIO_PIN_6

#define RADIO_DIO_0_PORT                          GPIOA
#define RADIO_DIO_0_PIN                           GPIO_PIN_10

#define RADIO_DIO_1_PORT                          GPIOB
#define RADIO_DIO_1_PIN                           GPIO_PIN_3

#define RADIO_DIO_2_PORT                          GPIOB
#define RADIO_DIO_2_PIN                           GPIO_PIN_5

#define RADIO_DIO_3_PORT                          GPIOB
#define RADIO_DIO_3_PIN                           GPIO_PIN_4

#ifdef RADIO_DIO_4 
#define RADIO_DIO_4_PORT                          GPIOA
#define RADIO_DIO_4_PIN                           GPIO_PIN_9
#endif

#ifdef RADIO_DIO_5 
#define RADIO_DIO_5_PORT                          GPIOC
#define RADIO_DIO_5_PIN                           GPIO_PIN_7
#endif



#define RADIO_ANT_SWITCH_PORT                     GPIOC
#define RADIO_ANT_SWITCH_PIN                      GPIO_PIN_1

#endif

/*  SPI MACRO redefinition */

#define SPI_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()


#define SPI1_AF                          GPIO_AF5_SPI1  

/* ADC MACRO redefinition */


#define ADC_READ_CHANNEL                 ADC_CHANNEL_3
#define ADCCLK_ENABLE()                 __HAL_RCC_ADC1_CLK_ENABLE() ;
#define ADCCLK_DISABLE()                 __HAL_RCC_ADC1_CLK_DISABLE() ;


/* --------------------------- RTC HW definition -------------------------------- */

#define RTC_OUTPUT       DBG_RTC_OUTPUT

/* --------------------------- USART HW definition -------------------------------*/
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE() 
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __USART2_RELEASE_RESET()


#define USARTx_TX_PIN                  GPIO_PIN_2
#define USARTx_TX_GPIO_PORT            GPIOA  
#define USARTx_TX_AF                   GPIO_AF7_USART2
#define USARTx_RX_PIN                  GPIO_PIN_3
#define USARTx_RX_GPIO_PORT            GPIOA 
#define USARTx_RX_AF                   GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel7

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn                DMA1_Channel7_IRQn
#define USARTx_DMA_TX_IRQHandler          DMA1_Channel7_IRQHandler

#define USARTx_Priority 0
#define USARTx_DMA_Priority 0

/* --------------------------- DEBUG redefinition -------------------------------*/

#define __HAL_RCC_DBGMCU_CLK_ENABLE()
#define __HAL_RCC_DBGMCU_CLK_DISABLE()

#define LED_Toggle( x )
#define LED_On( x )
#define LED_Off( x )

#ifdef __cplusplus
}
#endif

#endif /* __HW_CONF_L1_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
