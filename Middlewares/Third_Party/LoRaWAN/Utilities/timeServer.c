 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Generic lora driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    timeserver.c
  * @author  MCD Application Team
  * @brief   Time server infrastructure
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
#include <time.h>
#include "hw.h"
#include "timeServer.h"
//#include "low_power.h"


/*!
 * safely execute call back
 */
#define exec_cb( _callback_, context )     \
  do {                          \
      if( _callback_ == NULL )    \
      {                           \
        while(1);                 \
      }                           \
      else                        \
      {                           \
        _callback_( context );               \
      }                           \
  } while(0);                   



/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;
static SemaphoreHandle_t xTimeServerRunRes;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

/**
 * This function is initialize the OS resource for modbus master.
 * Note:The resource is define by OS.If you not use OS this function can be empty.
 *
 */
void vTimerServerOsResInit( void )
{
	if(xTimeServerRunRes == NULL) {
		/* Attempt to create a semaphore. */
		xTimeServerRunRes = xSemaphoreCreateBinary();
	}
}

/**
 * This function is take time Server running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be just return TRUE.
 *
 * @param lTimeOut the waiting time.
 *
 * @return resource taked result
 */
BOOL xTimeServerRunResTake( LONG lTimeOut )
{
	if(xTimeServerRunRes == NULL) {
		vTimerServerOsResInit();
	}
	if(xTimeServerRunRes == NULL)
		return TRUE;
    /*If waiting time is -1 .It will wait forever */
    return xSemaphoreTake(xTimeServerRunRes, lTimeOut) ? FALSE : TRUE ;
}

/**
 * This function is release Mobus Master running resource.
 * Note:The resource is define by Operating System.If you not use OS this function can be empty.
 *
 */
void vTimeServerRunResRelease( void )
{
	if(xTimeServerRunRes == NULL)
		return;
    /* release resource */
	xSemaphoreGive(xTimeServerRunRes);
}

void xTimeServerRunResGiveFromISR( void )
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/* 'Give' the semaphore to unblock the task, passing in the address of
	xHigherPriorityTaskWoken as the interrupt safe API function's
	pxHigherPriorityTaskWoken parameter. */

	xSemaphoreGiveFromISR(xTimeServerRunRes, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
  obj->Timestamp = 0;
  obj->ReloadValue = 0;
  obj->IsStarted = false;
  obj->IsNext2Expire = false;
  obj->Callback = callback;
  obj->Context = NULL;
  obj->Next = NULL;
//  vTimerServerOsResInit();
}

void TimerSetContext( TimerEvent_t *obj, void* context )
{
  obj->Context = context;
}

void TimerStart( TimerEvent_t *obj )
{
  uint32_t elapsedTime = 0;
  
//  BACKUP_PRIMASK();
//
//  DISABLE_IRQ( );
//  ENTER_CRITICAL_SECTION();
  uint32_t ulMask = taskENTER_CRITICAL_FROM_ISR();

  if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
  {
//      RESTORE_PRIMASK( );
//	EXIT_CRITICAL_SECTION();
	  taskEXIT_CRITICAL_FROM_ISR(ulMask);
    return;
  }
  obj->Timestamp = obj->ReloadValue;
  obj->IsStarted = true;
  obj->IsNext2Expire = false;

  if( TimerListHead == NULL )
  {
    HW_RTC_SetTimerContext( );
    TimerInsertNewHeadTimer( obj ); // insert a timeout at now+obj->Timestamp
  }
  else 
  {
    elapsedTime = HW_RTC_GetTimerElapsedTime( );
    obj->Timestamp += elapsedTime;
  
    if( obj->Timestamp < TimerListHead->Timestamp )
    {
      TimerInsertNewHeadTimer( obj);
    }
    else
    {
      TimerInsertTimer( obj);
    }
  }
//    RESTORE_PRIMASK( );
//  EXIT_CRITICAL_SECTION();
  taskEXIT_CRITICAL_FROM_ISR(ulMask);
}



bool TimerIsStarted( TimerEvent_t *obj )
{
  return obj->IsStarted;
}

void TimerIrqHandler( void )
{
  TimerEvent_t* cur;
  TimerEvent_t* next;
  
  uint32_t ulMask = taskENTER_CRITICAL_FROM_ISR();
  
  uint32_t old =  HW_RTC_GetTimerContext( );
  uint32_t now =  HW_RTC_SetTimerContext( );
  uint32_t DeltaContext = now - old; //intentionnal wrap around
  
  /* Update timeStamp based upon new Time Reference*/
  /* because delta context should never exceed 2^32*/
  if ( TimerListHead != NULL )
  {
    for (cur=TimerListHead; cur->Next != NULL; cur= cur->Next)
    {
      next =cur->Next;
      if (next->Timestamp > DeltaContext)
      {
        next->Timestamp -= DeltaContext;
      }
      else
      {
        next->Timestamp = 0 ;
      }
    }
  }
  
  /* execute imediately the alarm callback */
  if ( TimerListHead != NULL )
  {
    cur = TimerListHead;
    TimerListHead = TimerListHead->Next;
    cur->IsStarted = false;
    exec_cb( cur->Callback, cur->Context );
  }


  // remove all the expired object from the list
  while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp < HW_RTC_GetTimerElapsedTime(  )  ))
  {
   cur = TimerListHead;
   TimerListHead = TimerListHead->Next;
   cur->IsStarted = false;
   exec_cb( cur->Callback, cur->Context );
  }

  /* start the next TimerListHead if it exists AND NOT running */
  if( ( TimerListHead != NULL ) && ( TimerListHead->IsNext2Expire == false ) )
  {
    TimerSetTimeout( TimerListHead );
  }
  taskEXIT_CRITICAL_FROM_ISR(ulMask);
}

void TimerStop( TimerEvent_t *obj ) 
{
//  BACKUP_PRIMASK();
//
//  DISABLE_IRQ( );
//  ENTER_CRITICAL_SECTION();
  uint32_t ulMask = taskENTER_CRITICAL_FROM_ISR();
  TimerEvent_t* prev = TimerListHead;
  TimerEvent_t* cur = TimerListHead;

  // List is empty or the Obj to stop does not exist 
  if( ( TimerListHead == NULL ) || ( obj == NULL ) )
  {
//    RESTORE_PRIMASK( );
//    EXIT_CRITICAL_SECTION();
    taskEXIT_CRITICAL_FROM_ISR(ulMask);
    return;
  }

  obj->IsStarted = false;

  if( TimerListHead == obj ) // Stop the Head                  
  {
    if( TimerListHead->IsNext2Expire == true ) // The head is already running 
    {  
	  
      TimerListHead->IsNext2Expire = false;
      if( TimerListHead->Next != NULL )
      {
        TimerListHead = TimerListHead->Next;
        TimerSetTimeout( TimerListHead );
      }
      else
      {
        HW_RTC_StopAlarm( );
        TimerListHead = NULL;
      }
    }
    else // Stop the head before it is started
    {   
      if( TimerListHead->Next != NULL )   
      {
        TimerListHead = TimerListHead->Next;
      }
      else
      {
        TimerListHead = NULL;
      }
    }
  }
  else // Stop an object within the list
  {      
    while( cur != NULL )
    {
      if( cur == obj )
      {
        if( cur->Next != NULL )
        {
          cur = cur->Next;
          prev->Next = cur;
        }
        else
        {
          cur = NULL;
          prev->Next = cur;
        }
        break;
      }
      else
      {
        prev = cur;
        cur = cur->Next;
      }
    }   
  }
  
//  RESTORE_PRIMASK( );
//  EXIT_CRITICAL_SECTION();
  taskEXIT_CRITICAL_FROM_ISR(ulMask);
}  
  
void TimerReset( TimerEvent_t *obj )
{
  TimerStop( obj );
  TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
  uint32_t minValue = 0;
  uint32_t ticks = HW_RTC_ms2Tick( value );

  TimerStop( obj );

  minValue = HW_RTC_GetMinimumTimeout( );
  
  if( ticks < minValue )
  {
    ticks = minValue;
  }

  obj->Timestamp = ticks;
  obj->ReloadValue = ticks;
}

TimerTime_t TimerGetCurrentTime( void )
{
  uint32_t now = HW_RTC_GetTimerValue( );
  return  HW_RTC_Tick2ms(now);
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
  if ( past == 0 )
  {
    return 0;
  }
  uint32_t nowInTicks = HW_RTC_GetTimerValue( );
  uint32_t pastInTicks = HW_RTC_ms2Tick( past );
  /* intentional wrap around. Works Ok if tick duation below 1ms */
  return HW_RTC_Tick2ms( nowInTicks- pastInTicks );
}

static bool TimerExists( TimerEvent_t *obj )
{
  TimerEvent_t* cur = TimerListHead;

  while( cur != NULL )
  {
    if( cur == obj )
    {
      return true;
    }
    cur = cur->Next;
  }
  return false;
}
static void TimerSetTimeout( TimerEvent_t *obj )
{
  int32_t minTicks= HW_RTC_GetMinimumTimeout( );
  obj->IsNext2Expire = true; 

  // In case deadline too soon
  if(obj->Timestamp  < (HW_RTC_GetTimerElapsedTime(  ) + minTicks) )
  {
    obj->Timestamp = HW_RTC_GetTimerElapsedTime(  ) + minTicks;
  }
  HW_RTC_SetAlarm( obj->Timestamp );
}

TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
    return RtcTempCompensation( period, temperature );
}


static void TimerInsertTimer( TimerEvent_t *obj)
{
  TimerEvent_t* cur = TimerListHead;
  TimerEvent_t* next = TimerListHead->Next;

  while (cur->Next != NULL )
  {  
    if( obj->Timestamp  > next->Timestamp )
    {
        cur = next;
        next = next->Next;
    }
    else
    {
        cur->Next = obj;
        obj->Next = next;
        return;

    }
  }
  cur->Next = obj;
  obj->Next = NULL;
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
{
  TimerEvent_t* cur = TimerListHead;

  if( cur != NULL )
  {
    cur->IsNext2Expire = false;
  }

  obj->Next = cur;
  TimerListHead = obj;
  TimerSetTimeout( TimerListHead );
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
