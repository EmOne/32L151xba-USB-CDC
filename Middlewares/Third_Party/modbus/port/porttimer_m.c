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
#include "timers.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
#define TICK_PER_SECOND 1000U
static USHORT usT35TimeOut50us;
static TimerHandle_t timer;
static void prvvTIMERExpiredISR(void);
static void timer_timeout_ind(void* parameter);

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us)
{
    /* backup T35 ticks */
    usT35TimeOut50us = usTimeOut50us;

//    rt_timer_init(&timer, "master timer",
//                   timer_timeout_ind, /* bind timeout callback function */
//                   RT_NULL,
//                   (50 * usT35TimeOut50us) / (1000 * 1000 / RT_TICK_PER_SECOND) + 1,
//                   RT_TIMER_FLAG_ONE_SHOT); /* one shot */

    timer = xTimerCreate ("master timer",
    		pdMS_TO_TICKS((50 * usT35TimeOut50us) / (1000 * 1000 / TICK_PER_SECOND) + 1),
    		/* Setting uxAutoRealod to pdFALSE creates a one-shot software timer. */
    		pdFALSE,
    		/* This example does not use the timer id. */
    		0,
			timer_timeout_ind
    		);
    return TRUE;
}

void vMBMasterPortTimersT35Enable( void )
{
	const TickType_t timer_tick = pdMS_TO_TICKS((50 * usT35TimeOut50us) / (1000 * 1000 / TICK_PER_SECOND));

    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_T35);

    xTimerChangePeriod(timer, timer_tick, 0);

    xTimerStart( timer, 0 );
}

void vMBMasterPortTimersConvertDelayEnable()
{
	const TickType_t timer_tick = MB_MASTER_DELAY_MS_CONVERT * TICK_PER_SECOND / 1000;

    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);

    xTimerChangePeriod(timer, timer_tick, 0);

    xTimerStart( timer, 0 );
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
	const TickType_t timer_tick = MB_MASTER_TIMEOUT_MS_RESPOND * TICK_PER_SECOND / 1000;

    /* Set current timer mode, don't change it.*/
    vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);

    xTimerChangePeriod(timer, timer_tick, 0);

    xTimerStart( timer, 0 );
}

void vMBMasterPortTimersDisable()
{
    xTimerStop( timer, 0 );
}

void prvvTIMERExpiredISR(void)
{
    (void) pxMBMasterPortCBTimerExpired();
}

static void timer_timeout_ind(void* parameter)
{
    prvvTIMERExpiredISR();
}

#endif
