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
	/**
     * set 485 mode receive and transmit control IO
     * @note MODBUS_MASTER_RT_CONTROL_PIN_INDEX need be defined by user
     */
//    rt_pin_mode(MODBUS_MASTER_RT_CONTROL_PIN_INDEX, PIN_MODE_OUTPUT);
//
//    /* set serial name */
//    if (ucPORT == 1) {
//#if defined(RT_USING_UART1) || defined(RT_USING_REMAP_UART1)
//        extern struct rt_serial_device serial1;
//        serial = &serial1;
//#endif
//    } else if (ucPORT == 2) {
//#if defined(RT_USING_UART2)
//        extern struct rt_serial_device serial2;
//        serial = &serial2;
//#endif
//    } else if (ucPORT == 3) {
//#if defined(RT_USING_UART3)
//        extern struct rt_serial_device serial3;
//        serial = &serial3;
//#endif
//    }
//    /* set serial configure parameter */
//    serial->config.baud_rate = ulBaudRate;
//    serial->config.stop_bits = STOP_BITS_1;
//    switch(eParity){
//    case MB_PAR_NONE: {
//        serial->config.data_bits = DATA_BITS_8;
//        serial->config.parity = PARITY_NONE;
//        break;
//    }
//    case MB_PAR_ODD: {
//        serial->config.data_bits = DATA_BITS_9;
//        serial->config.parity = PARITY_ODD;
//        break;
//    }
//    case MB_PAR_EVEN: {
//        serial->config.data_bits = DATA_BITS_9;
//        serial->config.parity = PARITY_EVEN;
//        break;
//    }
//    }
//    /* set serial configure */
//    serial->ops->configure(serial, &(serial->config));
//
//    /* open serial device */
//    if (!serial->parent.open(&serial->parent,
//            RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX )) {
//        serial->parent.rx_indicate = serial_rx_ind;
//    } else {
//        return FALSE;
//    }

    /* software initialize */
//    rt_event_init(&event_serial, "master event", RT_IPC_FLAG_PRIO);
//    rt_thread_init(&thread_serial_soft_trans_irq,
//                   "master trans",
//                   serial_soft_trans_irq,
//                   RT_NULL,
//                   serial_soft_trans_irq_stack,
//                   sizeof(serial_soft_trans_irq_stack),
//                   10, 5);
//    rt_thread_startup(&thread_serial_soft_trans_irq);
    event_serial = xEventGroupCreate();
    osThreadDef(SerialTask, serial_soft_trans_irq, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
    thread_serial_soft_trans_irq = osThreadCreate(osThread(SerialTask), NULL);

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
    }
    else
    {
        /* stop serial transmit */
//        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
//                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_NO,
//                &recved_event);
    	recved_event = xEventGroupWaitBits( event_serial,
							EVENT_SERIAL_TRANS_START,
							pdTRUE,
							pdFALSE,
							portMAX_DELAY );
    	switch ( recved_event ) {
    	default:
    		break;
    	}
    }
}

void vMBMasterPortClose(void)
{
//    serial->parent.close(&(serial->parent));
	HAL_UART_DeInit(serial);
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
//    serial->parent.write(&(serial->parent), 0, &ucByte, 1);
    HAL_UART_Transmit_IT(serial, &ucByte, 1);
    return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
//    serial->parent.read(&(serial->parent), 0, pucByte, 1);
	HAL_UART_Receive_IT(serial, pucByte, 1);
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
    while (1)
    {
        /* waiting for serial transmit start */
    	recved_event =  xEventGroupWaitBits(event_serial,
    					EVENT_SERIAL_TRANS_START,
						pdFALSE,
    					pdTRUE,
						portMAX_DELAY);

    	switch ( recved_event ) {
		default:
			break;
		}
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
//static HAL_StatusTypeDef serial_rx_ind(rt_device_t dev, rt_size_t size)
{
    prvvUARTRxISR();
//    return HAL_OK;
}



#endif /*MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0*/
