/*!
 * \file      sx1272mb2das-board.c
 *
 * \brief     Target board SX1272MB2DAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
/**
  ******************************************************************************
  * @file    sx1272mb2das.c
  * @author  MCD Application Team
  * @brief   driver sx1272mb2das board 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "radio.h"
#include "sx1272.h"
#include "sx1272mb2das.h"


#define IRQ_HIGH_PRIORITY  5

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

void SX1272SetXO( uint8_t state );

uint32_t SX1272GetWakeTime( void );

void SX1272IoIrqInit( DioIrqHandler **irqHandlers );

uint8_t SX1272GetPaSelect( uint32_t channel );

void SX1272SetAntSwLowPower( bool status );

void SX1272SetRfTxPower( int8_t power );

/*!
 * \brief Controls the antenna switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] opMode Current radio operating mode
 */
void SX1272SetAntSw( uint8_t opMode );

/*!
 * \brief Initializes the RF Switch I/Os pins interface
 */
void SX1272AntSwInit( void );

/*!
 * \brief De-initializes the RF Switch I/Os pins interface
 *
 * \remark Needed to decrease the power consumption in MCU low power modes
 */
void SX1272AntSwDeInit( void );

static LoRaBoardCallback_t BoardCallbacks = { SX1272SetXO,
                                              SX1272GetWakeTime,
                                              SX1272IoIrqInit,
                                              SX1272SetRfTxPower,
                                              SX1272SetAntSwLowPower,
                                              SX1272SetAntSw};

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1272IoInit,
    SX1272IoDeInit,
    SX1272Init,
    SX1272GetStatus,
    SX1272SetModem,
    SX1272SetChannel,
    SX1272IsChannelFree,
    SX1272Random,
    SX1272SetRxConfig,
    SX1272SetTxConfig,
    SX1272CheckRfFrequency,
    SX1272GetTimeOnAir,
    SX1272Send,
    SX1272SetSleep,
    SX1272SetStby,
    SX1272SetRx,
    SX1272StartCad,
    SX1272SetTxContinuousWave,
    SX1272ReadRssi,
    SX1272Write,
    SX1272Read,
    SX1272WriteBuffer,
    SX1272ReadBuffer,
    SX1272SetMaxPayloadLength,
    SX1272SetPublicNetwork,
    SX1272GetWakeupTime
};

uint32_t SX1272GetWakeTime( void )
{
  return  BOARD_WAKEUP_TIME;
}

void SX1272SetXO( uint8_t state )
{
}
void SX1272IoInit( void )
{
  GPIO_InitTypeDef initStruct={0};
  
  SX1272BoardInit( &BoardCallbacks );
  
  initStruct.Mode = GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_PULLDOWN;
  initStruct.Speed = GPIO_SPEED_HIGH;

  HW_GPIO_Init( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, &initStruct );
  HW_GPIO_Init( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, &initStruct );
  HW_GPIO_Init( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, &initStruct );
  HW_GPIO_Init( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, &initStruct );
}

void SX1272IoIrqInit( DioIrqHandler **irqHandlers )
{
  HW_GPIO_SetIrq( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, IRQ_HIGH_PRIORITY, irqHandlers[0] );
  HW_GPIO_SetIrq( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, IRQ_HIGH_PRIORITY, irqHandlers[1] );
  HW_GPIO_SetIrq( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, IRQ_HIGH_PRIORITY, irqHandlers[2] );
  HW_GPIO_SetIrq( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, IRQ_HIGH_PRIORITY, irqHandlers[3] );
}

void SX1272IoDeInit( void )
{
  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode = GPIO_MODE_IT_RISING ;//GPIO_MODE_ANALOG;
  initStruct.Pull = GPIO_PULLDOWN;
  
  HW_GPIO_Init( RADIO_DIO_0_PORT, RADIO_DIO_0_PIN, &initStruct );
  HW_GPIO_Init( RADIO_DIO_1_PORT, RADIO_DIO_1_PIN, &initStruct );
  HW_GPIO_Init( RADIO_DIO_2_PORT, RADIO_DIO_2_PIN, &initStruct );
  HW_GPIO_Init( RADIO_DIO_3_PORT, RADIO_DIO_3_PIN, &initStruct );
}

void SX1272SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1272Read( REG_PACONFIG );
    paDac = SX1272Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1272GetPaSelect( SX1272.Settings.Channel );

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1272Write( REG_PACONFIG, paConfig );
    SX1272Write( REG_PADAC, paDac );
}

uint8_t SX1272GetPaSelect( uint32_t channel )
{
    return RF_PACONFIG_PASELECT_RFO;
}

void SX1272SetAntSwLowPower( bool status )
{
  //Ant Switch Controlled by SX1272 IC
	if (RadioIsActive != status) {
		RadioIsActive = status;

		if (status == false) {
			SX1272AntSwInit();
		} else {
			SX1272AntSwDeInit();
		}
	}
}

void SX1272AntSwInit( void )
{
	GPIO_InitTypeDef initStruct = { 0 };

	initStruct.Pull = GPIO_PULLUP;
	initStruct.Speed = GPIO_SPEED_HIGH;
	initStruct.Mode = GPIO_MODE_OUTPUT_PP;

	HW_GPIO_Init( RADIO_ANT_SWITCH_TX_GPIO_Port, RADIO_ANT_SWITCH_TX_Pin,
			&initStruct);
	HW_GPIO_Init( RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin,
			&initStruct);
	HW_GPIO_Write(RADIO_ANT_SWITCH_TX_GPIO_Port, RADIO_ANT_SWITCH_TX_Pin, GPIO_PIN_RESET);
	HW_GPIO_Write(RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, GPIO_PIN_SET);
//    GpioInit( &AntTx, RADIO_ANT_SWITCH_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
//    GpioInit( &AntRx, RADIO_ANT_SWITCH_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

void SX1272AntSwDeInit( void )
{
	GPIO_InitTypeDef initStruct = { 0 };

	initStruct.Mode = GPIO_MODE_ANALOG;
	initStruct.Pull = GPIO_NOPULL;

	HW_GPIO_Init( RADIO_ANT_SWITCH_TX_GPIO_Port, RADIO_ANT_SWITCH_TX_Pin,
			&initStruct);
	HW_GPIO_Init( RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin,
			&initStruct);

//    GpioInit( &AntTx, RADIO_ANT_SWITCH_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &AntRx, RADIO_ANT_SWITCH_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void SX1272SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
    	HW_GPIO_Write(RADIO_ANT_SWITCH_TX_GPIO_Port, RADIO_ANT_SWITCH_TX_Pin, GPIO_PIN_SET);
    	HW_GPIO_Write(RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, GPIO_PIN_RESET);
    	SX1272.RxTx = 1;
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
    	HW_GPIO_Write(RADIO_ANT_SWITCH_TX_GPIO_Port, RADIO_ANT_SWITCH_TX_Pin, GPIO_PIN_RESET);
    	HW_GPIO_Write(RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, GPIO_PIN_SET);
        SX1272.RxTx = 0;
        break;
    }
}

bool SX1272CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
