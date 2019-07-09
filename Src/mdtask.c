#if defined (STM32L151xBA)
#include "stm32l1xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"

uint8_t CpuUsageMajor, CpuUsageMinor;
USHORT  usModbusUserData[MB_PDU_SIZE_MAX];
UCHAR   ucModbusUserData[MB_PDU_SIZE_MAX];
__ALIGNED(portBYTE_ALIGNMENT)
extern USHORT   usSRegInBuf[];


void thread_entry_ModbusSlavePoll(void const * argument)
{
//  /* ABCDEF */
  usSRegInBuf[0] = 11;
  usSRegInBuf[1] = 22;
  usSRegInBuf[2] = 33;
  usSRegInBuf[3] = 44;
  usSRegInBuf[4] = 55;
  usSRegInBuf[5] = 66;
  usSRegInBuf[6] = 77;
  usSRegInBuf[7] = 88;

  eMBErrorCode eStatus =
  eMBInit(MB_RTU, 2, 3, 4800, MB_PAR_NONE );

  if (eStatus == MB_ENOERR) {
	 eStatus = eMBEnable();
  }

  while(1) {
    eMBPoll();
  }
}

void thread_entry_ModbusMasterPoll(void const * argument)
{ 
  
  eMBErrorCode eStatus =
		  eMBMasterInit(MB_RTU, 1, 4800, MB_PAR_NONE);

  if (eStatus == MB_ENOERR) {
	 eStatus = eMBEnable();
  }

  while(1) {
    eMBMasterPoll();
  }
}

void thread_entry_SysMonitor(void const * argument)
{
	eMBMasterReqErrCode    errorCode = MB_MRE_NO_ERR;
	uint16_t errorCount = 0;
	while (1)
	{
//		cpu_usage_get(&CpuUsageMajor, &CpuUsageMinor);
//		usSRegHoldBuf[S_HD_CPU_USAGE_MAJOR] = CpuUsageMajor;
//		usSRegHoldBuf[S_HD_CPU_USAGE_MINOR] = CpuUsageMinor;
//		LED_LED1_ON;
//		LED_LED2_ON;
//		rt_thread_delay(DELAY_SYS_RUN_LED);
//		LED_LED1_OFF;
//		LED_LED2_OFF;
//		rt_thread_delay(DELAY_SYS_RUN_LED);
//		IWDG_Feed(); //feed the dog
		//Test Modbus Master
		usModbusUserData[0] = (USHORT)(HAL_GetTick()/10);
		usModbusUserData[1] = (USHORT)(HAL_GetTick()%10);
		ucModbusUserData[0] = 0x1F;
//		errorCode = eMBMasterReqReadDiscreteInputs(1,3,8,RT_WAITING_FOREVER);
//		errorCode = eMBMasterReqWriteMultipleCoils(1,3,5,ucModbusUserData,RT_WAITING_FOREVER);
//		errorCode = eMBMasterReqWriteCoil(1,8,0xFF00,RT_WAITING_FOREVER);
//		errorCode = eMBMasterReqReadCoils(1,3,8,RT_WAITING_FOREVER);
		errorCode = eMBMasterReqReadInputRegister(1, 3, 8, -1);
//		errorCode = eMBMasterReqWriteHoldingRegister(1,3,usModbusUserData[0],RT_WAITING_FOREVER);
//		errorCode = eMBMasterReqWriteMultipleHoldingRegister(1,3,2,usModbusUserData,RT_WAITING_FOREVER);
//		errorCode = eMBMasterReqReadHoldingRegister(1,3,2,RT_WAITING_FOREVER);
//		errorCode = eMBMasterReqReadWriteMultipleHoldingRegister(1,3,2,usModbusUserData,5,2,RT_WAITING_FOREVER);
		//􏺼􏻇¼􏺳􏻶􏺴􏻭􏺴􏻎􏻊􏻽
		if (errorCode != MB_MRE_NO_ERR) {
			errorCount++;
		}
	}
}

