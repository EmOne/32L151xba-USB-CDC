#include "main.h"
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"
#include "cpu_utils.h"
//#include "util_console.h"
//#include "usbd_cdc_if.h"

//USHORT usModbusUserData[MB_PDU_SIZE_MAX];
//UCHAR ucModbusUserData[MB_PDU_SIZE_MAX];

__ALIGNED(portBYTE_ALIGNMENT)
#if MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0
extern USHORT usSRegInBuf[];
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

	for(;;) {
		eMBPoll();
	}
}
#endif

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
void thread_entry_ModbusMasterPoll(void const * argument) {
//	traceTASK_SWITCHED_IN()
//	;

	eMBErrorCode eStatus = eMBMasterInit(MB_RTU, 1, 4800, MB_PAR_NONE);

	if (eStatus == MB_ENOERR) {
		eStatus = eMBMasterEnable();
	}

	for (;;) {
		vTaskDelay(1);
		eMBMasterPoll();
	}
}

void thread_entry_Simulation(void const * argument) {
	eMBMasterReqErrCode errorCode = MB_MRE_NO_ERR;
	uint16_t errorCount = 0;

	UCHAR ucSndAddr = 1;
	USHORT usRegAddr = 0;
	USHORT usNRegs = 6;
	TickType_t tInterval = pdMS_TO_TICKS(1000);

//	traceTASK_SWITCHED_IN()
//	;

	for (;;) {
		vTaskDelay(tInterval);
//		CpuUsageMajor = osGetCPUUsage();
//		if (CpuUsageMajor > CpuUsageMinor) {
//			CpuUsageMinor = CpuUsageMajor;
//		}
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
//		usModbusUserData[0] = (USHORT) (HAL_GetTick() / 10);
//		usModbusUserData[1] = (USHORT) (HAL_GetTick() % 10);
//		ucModbusUserData[0] = 0x1F;
//		errorCode = eMBMasterReqReadDiscreteInputs(1,3,8,-1);
//		errorCode = eMBMasterReqWriteMultipleCoils(1,3,5,ucModbusUserData,-1);
//		errorCode = eMBMasterReqWriteCoil(1,8,0xFF00,-1);
//		errorCode = eMBMasterReqReadCoils(1,3,8,-1);
		errorCode = eMBMasterReqReadInputRegister(ucSndAddr, usRegAddr, usNRegs, 1000);
//		errorCode = eMBMasterReqWriteHoldingRegister(1,3,usModbusUserData[0],-1);
//		errorCode = eMBMasterReqWriteMultipleHoldingRegister(1,3,2,usModbusUserData,-1);
//		errorCode = eMBMasterReqReadHoldingRegister(1,3,2,-1);
//		errorCode = eMBMasterReqReadWriteMultipleHoldingRegister(1,3,2,usModbusUserData,5,2,-1);

		if (errorCode != MB_MRE_NO_ERR) {
			errorCount++;
		} 
//                else 
//                {
////			CDC_Transmit_FS((uint8_t *) &usMRegInBuf[ucSndAddr - 1][0], usNRegs);
//			PRINTF("Temp=%d\r\n", usMRegInBuf[ucSndAddr - 1][0]);
//		}
	}
}
#endif
