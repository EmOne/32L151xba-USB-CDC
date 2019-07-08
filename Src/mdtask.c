#if defined (STM32L151xBA)
#include "stm32l1xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"


void ModbusRTUTask(void const * argument)
{ 
//  /* ABCDEF */
//  usRegInputBuf[0] = 11;
//  usRegInputBuf[1] = 22;
//  usRegInputBuf[2] = 33;
//  usRegInputBuf[3] = 44;
//  usRegInputBuf[4] = 55;
//  usRegInputBuf[5] = 66;
//  usRegInputBuf[6] = 77;
//  usRegInputBuf[7] = 88;
  
  eMBErrorCode eStatus =
#if defined (MB_MASTER_RTU_ENABLED)
		  eMBMasterInit(MB_RTU, 1, 4800, MB_PAR_NONE);
#else
  	  eMBInit(MB_RTU, 1, 3, 4800, MB_PAR_NONE );
#endif

  if (eStatus == MB_ENOERR) {
#if defined (MB_MASTER_RTU_ENABLED)
	 eStatus = eMBMasterEnable();
#else
	 eStatus = eMBEnable();
#endif
  }

  while(1) {
#if defined (MB_MASTER_RTU_ENABLED)
    eMBMasterPoll();
#else
    eMBPoll();
#endif
  }
}
