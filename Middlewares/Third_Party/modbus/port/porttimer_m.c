/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 * Maintain by: Anol Paisal <anol.p@emone.co.th>
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "FreeRTOS.h"
#ifdef SW_TIMER
#include "timers.h"
#endif
#include "semphr.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"

#undef SW_TIMER

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
#define TICK_PER_SECOND 1000U
static USHORT usT35TimeOut50us;
uint16_t downcounterMaster = 0;
//static SemaphoreHandle_t xMasterTimerRunRes;
#ifdef SW_TIMER
static TimerHandle_t timer;
#else
extern TIM_HandleTypeDef htim7;
#endif
void prvvMastesTIMERExpiredISR(void);

/* ----------------------- static functions ---------------------------------*/
//static void prvvTIMERExpiredISR(void);
static void timer_timeout_ind(void* parameter);
/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us)
{
//	xMasterTimerRunRes = xSemaphoreCreateBinary();
    /* backup T35 ticks */
    usT35TimeOut50us = usTimeOut50us;
#ifdef SW_TIMER
////    rt_timer_init(&timer, "master timer",
////                   timer_timeout_ind, /* bind timeout callback function */
////                   RT_NULL,
////                   (50 * usT35TimeOut50us) / (1000 * 1000 / RT_TICK_PER_SECOND) + 1,
////                   RT_TIMER_FLAG_ONE_SHOT); /* one shot */
//
    timer = xTimerCreate ("master timer",
    		pdMS_TO_TICKS((50 * usT35TimeOut50us) / (1000 * 1000 / TICK_PER_SECOND) + 1),
    		/* Setting uxAutoRealod to pdFALSE creates a one-shot software timer. */
    		pdFALSE,
    		/* This example does not use the timer id. */
    		0,
			timer_timeout_ind
    		);
#else
	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) - 1;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 50 - 1;

	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		return FALSE;
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		return FALSE;
	}
	vMBMasterPortTimersDisable();
#endif
	return TRUE;
}

void vMBMasterPortTimersT35Enable( void )
{
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
#ifdef SW_TIMER
	const TickType_t timer_tick = pdMS_TO_TICKS((50 * usT35TimeOut50us) / (1000 * 1000 / TICK_PER_SECOND) + 1);
    xTimerChangePeriod(timer, timer_tick, 0);

    xTimerStart( timer, 0 );
#else
	downcounterMaster = usT35TimeOut50us;
    HAL_TIM_Base_Start_IT(&htim7);
#endif
}

void vMBMasterPortTimersConvertDelayEnable()
{
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
#ifdef SW_TIMER
    const TickType_t timer_tick = MB_MASTER_DELAY_MS_CONVERT * TICK_PER_SECOND / 1000;
    xTimerChangePeriod(timer, timer_tick, 0);

    xTimerStart( timer, 0 );
#else
    htim7.Init.Period = MB_MASTER_DELAY_MS_CONVERT - 1;
	HAL_TIM_Base_Init(&htim7);
	downcounterMaster = usT35TimeOut50us;
	HAL_TIM_Base_Start_IT(&htim7);
#endif
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);
#ifdef SW_TIMER
	const TickType_t timer_tick = MB_MASTER_TIMEOUT_MS_RESPOND * TICK_PER_SECOND / 1000;
    xTimerChangePeriod(timer, timer_tick, 0);
//
    xTimerStart( timer, 0 );
#else
    htim7.Init.Period = MB_MASTER_TIMEOUT_MS_RESPOND - 1;
	HAL_TIM_Base_Init(&htim7);
    HAL_TIM_Base_Start_IT(&htim7);
#endif
}

void vMBMasterPortTimersDisable()
{
#ifdef SW_TIMER
    xTimerStop( timer, 0 );
#else
    HAL_TIM_Base_Stop_IT(&htim7);
#endif
}

void prvvMastesTIMERExpiredISR(void)
{
	static portBASE_TYPE xTaskSwitch = pdFALSE;
	if (!--downcounterMaster)
		xTaskSwitch |= pxMBMasterPortCBTimerExpired();
}

#ifdef SW_TIMER
static void timer_timeout_ind(void* parameter)
{
	prvvMastesTIMERExpiredISR();
}
#endif
#endif
