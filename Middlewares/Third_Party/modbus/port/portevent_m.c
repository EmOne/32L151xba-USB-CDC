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
 * File: $Id: portevent_m.c v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 * Maintain by: Anol Paisal <anol.p@emone.co.th>
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "timers.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Defines ------------------------------------------*/
/* ----------------------- Variables ----------------------------------------*/
static SemaphoreHandle_t xMasterRunRes;
static EventGroupHandle_t xMasterOsEvent;
static eMBMasterEventType eQueuedMasterEvent;
static BOOL     xEventMasterInQueue;
/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBMasterPortEventInit( void )
{
	if(xMasterOsEvent == NULL)
    {
		xMasterOsEvent = xEventGroupCreate();
    }
	xEventMasterInQueue = FALSE;
	return TRUE;
}

BOOL
xMBMasterPortEventPost( eMBMasterEventType eEvent )
{
	xEventMasterInQueue = TRUE;
	eQueuedMasterEvent = eEvent;
	xEventGroupSetBits(xMasterOsEvent, eEvent);
    return TRUE;
}

BOOL
xMBMasterPortEventPostFromISR( eMBMasterEventType eEvent )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xEventMasterInQueue = TRUE;
	eQueuedMasterEvent = eEvent;
	xEventGroupSetBitsFromISR(xMasterOsEvent, eEvent, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    return TRUE;
}


BOOL
xMBMasterPortEventGet( eMBMasterEventType * eEvent )
{
	BOOL xEventHappened = FALSE;
//	const EventBits_t xBitsToWaitFor = ( EV_MASTER_READY | EV_MASTER_FRAME_RECEIVED | EV_MASTER_EXECUTE |
//            EV_MASTER_FRAME_SENT | EV_MASTER_ERROR_PROCESS );
    /* waiting forever OS event */
//	EventBits_t recvedEvent = xEventGroupWaitBits(xMasterOsEvent,
//			xBitsToWaitFor,
//			pdTRUE,
//			pdFALSE,
//			portMAX_DELAY);


//	xEventGroupSetBits(xMasterOsEvent, eQueuedMasterEvent);
	if (xEventMasterInQueue) {
		*eEvent = eQueuedMasterEvent;
		xEventMasterInQueue = FALSE;
		xEventHappened = TRUE;
	}
//	return xEventHappened;
	EventBits_t recvedEvent =xEventGroupGetBitsFromISR(xMasterOsEvent);
//    /* the enum type couldn't convert to int type */
    switch (recvedEvent)
    {
    case EV_MASTER_READY:
        *eEvent = EV_MASTER_READY;
        break;
    case EV_MASTER_FRAME_RECEIVED:
        *eEvent = EV_MASTER_FRAME_RECEIVED;
        break;
    case EV_MASTER_EXECUTE:
        *eEvent = EV_MASTER_EXECUTE;
        break;
    case EV_MASTER_FRAME_SENT:
        *eEvent = EV_MASTER_FRAME_SENT;
        break;
    case EV_MASTER_ERROR_PROCESS:
        *eEvent = EV_MASTER_ERROR_PROCESS;
        break;
    }
    return TRUE;
}
/**
 * This function is initialize the OS resource for modbus master.
 * Note:The resource is define by OS.If you not use OS this function can be empty.
 *
 */
void vMBMasterOsResInit( void )
{
	if(xMasterRunRes == NULL) {
		/* Attempt to create a semaphore. */
		xMasterRunRes = xSemaphoreCreateBinary();
	}
}

/**
 * This function is take Mobus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be just return TRUE.
 *
 * @param lTimeOut the waiting time.
 *
 * @return resource taked result
 */
BOOL xMBMasterRunResTake( LONG lTimeOut )
{
	if(xMasterRunRes == NULL) {
		vMBMasterOsResInit();
	}
    /*If waiting time is -1 .It will wait forever */
    return xSemaphoreTake(xMasterRunRes, lTimeOut) ? FALSE : TRUE ;
}

/**
 * This function is take Mobus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be just return TRUE.
 *
 * @param lTimeOut the waiting time.
 *
 * @return resource taked result
 */
void xMBMasterRunResGiveFromISR( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* 'Give' the semaphore to unblock the task, passing in the address of
	xHigherPriorityTaskWoken as the interrupt safe API function's
	pxHigherPriorityTaskWoken parameter. */

	xSemaphoreGiveFromISR(xMasterRunRes, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 * This function is release Mobus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be empty.
 *
 */
void vMBMasterRunResRelease( void )
{
    /* release resource */
	xSemaphoreGive(xMasterRunRes);
}

/**
 * This is modbus master respond timeout error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBRespondTimeout(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
	xMBMasterPortEventPost(EV_MASTER_ERROR_RESPOND_TIMEOUT);
    /* You can add your code under here. */

}

/**
 * This is modbus master receive data error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBReceiveData(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
	xMBMasterPortEventPost(EV_MASTER_ERROR_RECEIVE_DATA);
    /* You can add your code under here. */

}

/**
 * This is modbus master execute function error process callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 * @param ucDestAddress destination salve address
 * @param pucPDUData PDU buffer data
 * @param ucPDULength PDU buffer length
 *
 */
void vMBMasterErrorCBExecuteFunction(UCHAR ucDestAddress, const UCHAR* pucPDUData,
        USHORT ucPDULength) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
	xMBMasterPortEventPost(EV_MASTER_ERROR_EXECUTE_FUNCTION);
    /* You can add your code under here. */

}

/**
 * This is modbus master request process success callback function.
 * @note There functions will block modbus master poll while execute OS waiting.
 * So,for real-time of system.Do not execute too much waiting process.
 *
 */
void vMBMasterCBRequestSuccess( void ) {
    /**
     * @note This code is use OS's event mechanism for modbus master protocol stack.
     * If you don't use OS, you can change it.
     */
	xMBMasterPortEventPost(EV_MASTER_PROCESS_SUCESS);
    /* You can add your code under here. */

}

/**
 * This function is wait for modbus master request finish and return result.
 * Waiting result include request process success, request respond timeout,
 * receive data error and execute function error.You can use the above callback function.
 * @note If you are use OS, you can use OS's event mechanism. Otherwise you have to run
 * much user custom delay for waiting.
 *
 * @return request error code
 */
eMBMasterReqErrCode eMBMasterWaitRequestFinish( void ) {
//    eMBMasterReqErrCode    eErrStatus = MB_MRE_NO_ERR;
//    const EventBits_t xBitsToWaitFor = ( EV_MASTER_PROCESS_SUCESS | EV_MASTER_ERROR_RESPOND_TIMEOUT
//            | EV_MASTER_ERROR_RECEIVE_DATA
//            | EV_MASTER_ERROR_EXECUTE_FUNCTION );
//    /* waiting for OS event */
//    EventBits_t recvedEvent = xEventGroupWaitBits(xMasterOsEvent,
//            xBitsToWaitFor,
//			pdTRUE,
//			pdFALSE,
//			portMAX_DELAY);
//    while(1)
		switch (eQueuedMasterEvent)
		{
//		case EV_MASTER_READY:
//		case EV_MASTER_FRAME_RECEIVED:
//		case EV_MASTER_EXECUTE:
//		case EV_MASTER_FRAME_SENT:
		case EV_MASTER_PROCESS_SUCESS:
		{
			eQueuedMasterEvent &= ~EV_MASTER_PROCESS_SUCESS;
			return MB_MRE_NO_ERR;

		}
		case EV_MASTER_ERROR_RESPOND_TIMEOUT:
		{
			eQueuedMasterEvent &= ~EV_MASTER_ERROR_RESPOND_TIMEOUT;
			return MB_MRE_TIMEDOUT;
		}
		case EV_MASTER_ERROR_RECEIVE_DATA:
		{
			eQueuedMasterEvent &= ~EV_MASTER_ERROR_RECEIVE_DATA;
			return MB_MRE_REV_DATA;
		}
		case EV_MASTER_ERROR_EXECUTE_FUNCTION:
		{
			eQueuedMasterEvent &= ~EV_MASTER_ERROR_EXECUTE_FUNCTION;
			return MB_MRE_EXE_FUN;
		}
		default:
			return MB_MRE_NO_ERR;
		}

}

#endif
