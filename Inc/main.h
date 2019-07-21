/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RADIO_ANT_SWITCH_RX_Pin GPIO_PIN_13
#define RADIO_ANT_SWITCH_RX_GPIO_Port GPIOC
#define RX_TX_Pin GPIO_PIN_1
#define RX_TX_GPIO_Port GPIOA
#define RADIO_RESET_Pin GPIO_PIN_2
#define RADIO_RESET_GPIO_Port GPIOA
#define RADIO_ANT_SWITCH_TX_Pin GPIO_PIN_4
#define RADIO_ANT_SWITCH_TX_GPIO_Port GPIOA
#define RADIO_SCK_Pin GPIO_PIN_5
#define RADIO_SCK_GPIO_Port GPIOA
#define RADIO_MISO_Pin GPIO_PIN_6
#define RADIO_MISO_GPIO_Port GPIOA
#define RADIO_MOSI_Pin GPIO_PIN_7
#define RADIO_MOSI_GPIO_Port GPIOA
#define RADIO_NSS_Pin GPIO_PIN_0
#define RADIO_NSS_GPIO_Port GPIOB
#define RADIO_DIO_0_Pin GPIO_PIN_1
#define RADIO_DIO_0_GPIO_Port GPIOB
#define RADIO_DIO_1_Pin GPIO_PIN_10
#define RADIO_DIO_1_GPIO_Port GPIOB
#define RADIO_DIO_2_Pin GPIO_PIN_11
#define RADIO_DIO_2_GPIO_Port GPIOB
#define RADIO_DIO_5_Pin GPIO_PIN_4
#define RADIO_DIO_5_GPIO_Port GPIOB
#define RADIO_DIO_4_Pin GPIO_PIN_5
#define RADIO_DIO_4_GPIO_Port GPIOB
#define RADIO_DIO_3_Pin GPIO_PIN_7
#define RADIO_DIO_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

extern uint8_t   ucSDiscInBuf[]  ;
extern uint8_t   ucSCoilBuf[]    ;
extern uint16_t   usSRegInBuf[]   ;
extern uint16_t   usSRegHoldBuf[] ;

extern uint16_t CpuUsageMajor;
extern uint16_t CpuUsageMinor;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
