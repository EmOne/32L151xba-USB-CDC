/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
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
 * File: $Id: port.h,v 1.1 2006/08/22 21:35:13 wolti Exp $
 * Maintain by : Anol Paisal <anol.p@emone.co.th> 2019
 */

#ifndef _PORT_H
#define _PORT_H

#include "main.h"
#include "cmsis_os.h"
//#if defined (STM32L151xBA)
//#include "stm32l1xx_hal.h"
//#else
//#include "stm32f4xx_hal.h"
//#endif
#include "mbconfig.h"

#include <assert.h>
#include <inttypes.h>


#define	INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define	PR_END_EXTERN_C             }

#define ENTER_CRITICAL_SECTION() portENTER_CRITICAL()//uint32_t mask; BoardCriticalSectionBegin( &mask ) //
#define EXIT_CRITICAL_SECTION()  portEXIT_CRITICAL()//BoardCriticalSectionEnd( &mask ) //

/*
 * ============================================================================
 * Following functions must be implemented inside the specific platform
 * port.c file.
 * ============================================================================
 */
/*!
 * Disable interrupts, begins critical section
 *
 * \param [IN] mask Pointer to a variable where to store the CPU IRQ mask
 */
INLINE void BoardCriticalSectionBegin( uint32_t *mask ) ;

/*!
 * Ends critical section
 *
 * \param [IN] mask Pointer to a variable where the CPU IRQ mask was stored
 */
INLINE void BoardCriticalSectionEnd( uint32_t *mask ) ;

typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE            1
#endif

#ifndef FALSE
#define FALSE           0
#endif

#endif
