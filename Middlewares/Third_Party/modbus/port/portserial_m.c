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
 * File: $Id: portserial_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions $
 * Maintain by: Anol Paisal <anol.p@emone.co.th>
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "event_groups.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

//#include "rtdevice.h"
//#include "bsp.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Static variables ---------------------------------*/
//__ALIGNED(portBYTE_ALIGNMENT)
/* software simulation serial transmit IRQ handler thread stack */
//static uint16_t serial_soft_trans_irq_stack = 512;
/* software simulation serial transmit IRQ handler thread */
static osThreadId thread_serial_soft_trans_irq;
/* serial event */
static EventGroupHandle_t event_serial;
/* modbus master serial device */
static UART_HandleTypeDef *serial;

/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);
//static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size);
static void serial_soft_trans_irq(void const * parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{

	if(event_serial == NULL)
		event_serial = xEventGroupCreate();

	if (ucPORT == 1) {
		extern UART_HandleTypeDef huart1;
		serial = (UART_HandleTypeDef *) &huart1;
	} else if (ucPORT == 2) {
//		extern UART_HandleTypeDef huart2;
//		serial = (UART_HandleTypeDef *) &huart2;
	} else if (ucPORT == 3) {
//		extern UART_HandleTypeDef huart3;
//		serial = (UART_HandleTypeDef *) &huart3;
	} else {

	}

	serial->Init.BaudRate = ulBaudRate;
	serial->Init.StopBits = UART_STOPBITS_1;

	switch(eParity){
	case MB_PAR_NONE: {
		serial->Init.WordLength = UART_WORDLENGTH_8B;
		serial->Init.Parity = UART_PARITY_NONE;
		break;
	}
	case MB_PAR_ODD: {
		serial->Init.WordLength = UART_WORDLENGTH_9B;
		serial->Init.Parity = UART_PARITY_ODD;
		break;
	}
	case MB_PAR_EVEN: {
		serial->Init.WordLength = UART_WORDLENGTH_9B;
		serial->Init.Parity = UART_PARITY_EVEN;
		break;
	}
	}
	if (HAL_UART_Init(serial) != HAL_OK) {

	}


//    /* open serial device */
//    if (!serial->parent.open(&serial->parent,
//            RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX )) {
//        serial->parent.rx_indicate = serial_rx_ind;
//    } else {
//        return FALSE;
//    }

    /* software initialize */
//    rt_thread_init(&thread_serial_soft_trans_irq,
//                   "master trans",
//                   serial_soft_trans_irq,
//                   RT_NULL,
//                   serial_soft_trans_irq_stack,
//                   sizeof(serial_soft_trans_irq_stack),
//                   10, 5);
//    rt_thread_startup(&thread_serial_soft_trans_irq);
//    osThreadDef(SerialTask, serial_soft_trans_irq, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
//    thread_serial_soft_trans_irq = osThreadCreate(osThread(SerialTask), NULL);

    return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
	EventBits_t recved_event;
    if (xRxEnable)
    {
        /* enable RX interrupt */
//        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    	__HAL_UART_ENABLE_IT(serial, UART_IT_RXNE);
        /* switch 485 to receive mode */
//        rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_LOW);
        HAL_GPIO_WritePin(RX_TX_GPIO_Port, RX_TX_Pin, GPIO_PIN_RESET);
    }
    else
    {
        /* switch 485 to transmit mode */
//        rt_pin_write(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_HIGH);
        HAL_GPIO_WritePin(RX_TX_GPIO_Port, RX_TX_Pin, GPIO_PIN_SET);
        /* disable RX interrupt */
//        serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        __HAL_UART_DISABLE_IT(serial, UART_IT_RXNE);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
//        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
        xEventGroupSetBits(event_serial, EVENT_SERIAL_TRANS_START);
        __HAL_UART_ENABLE_IT(serial, UART_IT_TXE);
//        prvvUARTTxReadyISR();
    }
    else
    {
        /* stop serial transmit */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
//                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO,
//                &recved_event);
//    	recved_event = xEventGroupWaitBits( event_serial,
//							EVENT_SERIAL_TRANS_START,
//							pdFALSE,
//							pdFALSE,
//							portMAX_DELAY );
    	__HAL_UART_DISABLE_IT(serial, UART_IT_TXE);
//    	switch ( recved_event ) {
//    	default:
//    		break;
//    	}
    }
}

void vMBMasterPortClose(void)
{
	HAL_UART_DeInit(serial);
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
	return (HAL_OK == HAL_UART_Transmit(serial, (uint8_t*)&ucByte, 1, 10));
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
	 *pucByte = (uint8_t)(serial->Instance->DR & (uint8_t)0x00FF);
    return TRUE;
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
    pxMBMasterFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
    pxMBMasterFrameCBByteReceived();
}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
static void serial_soft_trans_irq(void const * parameter) {
	EventBits_t recved_event;
    for (;;)
    {
        /* waiting for serial transmit start */
    	recved_event =  xEventGroupWaitBits(event_serial,
    					EVENT_SERIAL_TRANS_START,
						pdFALSE,
    					pdTRUE,
						portMAX_DELAY);

        /* execute modbus callback */
        prvvUARTTxReadyISR();
    }
}

/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    prvvUARTRxISR();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	prvvUARTTxReadyISR();
}

#endif /*MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0*/
