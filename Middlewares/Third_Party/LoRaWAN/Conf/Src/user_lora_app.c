/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   this is the main!
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

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "utilities.h"
#include "user_mb_app.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*!
 * CAYENNE_LPP is myDevices Application server.
 */
//#define CAYENNE_LPP
#define LPP_DATATYPE_DIGITAL_INPUT  0x0
#define LPP_DATATYPE_DIGITAL_OUTPUT 0x1
#define LPP_DATATYPE_HUMIDITY       0x68
#define LPP_DATATYPE_TEMPERATURE    0x67
#define LPP_DATATYPE_BAROMETER      0x73
#define LPP_APP_PORT 99
/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_OFF
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE 	DR_5
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_C
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           242
/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
//static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };
lora_AppData_t AppData={ AppDataBuff,  0 ,0 };

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* call back when LoRa endNode has received a frame*/
static void LORA_RxData( lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined( void );

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass ( DeviceClass_t Class );

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded ( void );

/* LoRa endNode send request*/
static void Send( void* context );

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent( void* context );

/* tx timer callback function*/
static void LoraMacProcessNotify( void );

/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { HW_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify};
LoraFlagStatus LoraMacProcessRequest=LORA_RESET;
LoraFlagStatus AppProcessRequest=LORA_RESET;
/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

static TimerEvent_t TxTimer;

#ifdef USE_B_L072Z_LRWAN1
/*!
 * Timer to handle the application Tx Led to toggle
 */
static TimerEvent_t TxLedTimer;
static void OnTimerLedEvent( void* context );
#endif
/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {LORAWAN_ADR_STATE,
                                    LORAWAN_DEFAULT_DATA_RATE,
                                    LORAWAN_PUBLIC_NETWORK};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void thread_LoRaPoll(void const * argument)
{
//	traceTASK_SWITCHED_IN()
//	;
  /* STM32 HAL library initialization*/
//  HAL_Init();

  /* Configure the system clock*/
//  SystemClock_Config();

  /* Configure the debug mode*/
//  DBG_Init();

  /* Configure the hardware*/
  HW_Init();

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id , LPM_Disable );

  PRINTF("VERSION: %X\n\r", VERSION);

  /* Configure the Lora Stack*/
  LORA_Init( &LoRaMainCallbacks, &LoRaParamInit);

  LORA_Join();

  LoraStartTx( TX_ON_EVENT) ;

  for(;;)
  {
	osDelay(100);
//	if(xTimeServerRunResTake(portMAX_DELAY) == pdPASS)
//	if ( xMBMasterRunResTake( portMAX_DELAY ) == pdPASS )
	{

		if (AppProcessRequest==LORA_SET)
		{
		  /*reset notification flag*/
		  AppProcessRequest=LORA_RESET;
		  /*Send*/
		  Send( NULL );

		}
		if (LoraMacProcessRequest==LORA_SET)
		{
		  /*reset notification flag*/
		  LoraMacProcessRequest=LORA_RESET;
		  LoRaMacProcess( );

		}
		/*If a flag is set at this point, mcu must not enter low power and must loop*/
//	    DISABLE_IRQ( );
//	    CRITICAL_SECTION_BEGIN( );
//
//	    /* if an interrupt has occurred after DISABLE_IRQ, it is kept pending
//	     * and cortex will not enter low power anyway  */
	    if ((LoraMacProcessRequest!=LORA_SET) && (AppProcessRequest!=LORA_SET))
	    {
//	#ifndef LOW_POWER_DISABLE
//	      LPM_EnterLowPower( );
//	#endif
//	    	vTimeServerRunResRelease();
//	    	vMBMasterRunResRelease( );
	    	vTaskSuspend( NULL );

	    }
//
//	    ENABLE_IRQ();
//	    CRITICAL_SECTION_END( );

	}
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */
  }
}


void LoraMacProcessNotify(void)
{
  LoraMacProcessRequest=LORA_SET;
}


static void LORA_HasJoined( void )
{
#if( OVER_THE_AIR_ACTIVATION != 0 )
  PRINTF("JOINED\n\r");
#endif
  LORA_RequestClass( LORAWAN_DEFAULT_CLASS );
}

static void Send( void* context )
{
  /* USER CODE BEGIN 3 */
  uint8_t len = 0; 
  uint32_t i = 0;
//  uint8_t batteryLevel;
  
  if ( LORA_JoinStatus () != LORA_SET)
  {
    /*Not joined, try again later*/
    LORA_Join();
    return;
  }

  TVL1(PRINTF("SEND REQUEST\n\r"););
 
//  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */
  AppData.Buff[i++] = (uint8_t ) user_setting.slave_id;
  AppData.Buff[i++] = HW_GetBatteryLevel( );
//  AppData.Buff[i++] = HW_GetTemperatureLevel();
  AppData.Buff[i++] = (uint8_t ) user_setting.reg_addr;
  len = sizeof(USHORT) * user_setting.qty;
  AppData.Buff[i++] = len;
  memcpy1(AppData.Buff + i, &usMRegInBuf[user_setting.slave_id - 1], len);
  i += len;

  AppData.BuffSize = i;
  AppData.Port = LORAWAN_APP_PORT;
  LORA_send( &AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);

  /* USER CODE END 3 */
}


static void LORA_RxData( lora_AppData_t *AppData )
{
  /* USER CODE BEGIN 4 */
  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case 3:
    /*this port switches the class*/
    if( AppData->BuffSize == 1 )
    {
      switch (  AppData->Buff[0] )
      {
        case 0:
        {
          LORA_RequestClass(CLASS_A);
          break;
        }
        case 1:
        {
          LORA_RequestClass(CLASS_B);
          break;
        }
        case 2:
        {
          LORA_RequestClass(CLASS_C);
          break;
        }
        default:
          break;
      }
    }
    break;
    case LORAWAN_APP_PORT:
    if( AppData->BuffSize == 1 )
    {
      AppLedStateOn = AppData->Buff[0] & 0x01;
      if ( AppLedStateOn == RESET )
      {
        PRINTF("LED OFF\n\r");
//        LED_Off( LED_BLUE ) ;
      }
      else
      {
        PRINTF("LED ON\n\r");
//        LED_On( LED_BLUE ) ;
      }
    }
    break;
  case LPP_APP_PORT:
  {
    AppLedStateOn= (AppData->Buff[2] == 100) ?  0x01 : 0x00;
    if ( AppLedStateOn == RESET )
    {
      PRINTF("LED OFF\n\r");
//      LED_Off( LED_BLUE ) ;

    }
    else
    {
      PRINTF("LED ON\n\r");
//      LED_On( LED_BLUE ) ;
    }
    break;
  }
  default:
    break;
  }
  /* USER CODE END 4 */
}

static void OnTxTimerEvent( void* context )
{
  /*Wait for next tx slot*/
  TimerStart( &TxTimer);

  AppProcessRequest=LORA_SET;
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit( &TxTimer, OnTxTimerEvent );
    TimerSetValue( &TxTimer,  APP_TX_DUTYCYCLE);
    OnTxTimerEvent( NULL );
  }
  else
  {
    /* send everytime button is pushed */
//    GPIO_InitTypeDef initStruct={0};
//
//    initStruct.Mode =GPIO_MODE_IT_RISING;
//    initStruct.Pull = GPIO_PULLUP;
//    initStruct.Speed = GPIO_SPEED_HIGH;
//
//    HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
//    HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 0, Send );
  }
}

static void LORA_ConfirmClass ( DeviceClass_t Class )
{
  PRINTF("switch to class %c done\n\r","ABC"[Class] );

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded ( void )
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send( &AppData, LORAWAN_UNCONFIRMED_MSG);
}

#ifdef USE_B_L072Z_LRWAN1
static void OnTimerLedEvent( void* context )
{
  LED_Off( LED_RED1 ) ;
}
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
