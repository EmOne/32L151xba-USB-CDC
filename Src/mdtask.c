#include "main.h"
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"
#include "cpu_utils.h"

USHORT usModbusUserData[MB_PDU_SIZE_MAX];
UCHAR ucModbusUserData[MB_PDU_SIZE_MAX];

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
extern USHORT usMRegInBuf[];
void thread_entry_ModbusMasterPoll(void const * argument) {
	eMBErrorCode eStatus = eMBMasterInit(MB_RTU, 1, 4800, MB_PAR_NONE);

	if (eStatus == MB_ENOERR) {
		eStatus = eMBMasterEnable();
	}

	for (;;) {
		eMBMasterPoll();
	}
}

void thread_entry_Simulation(void const * argument) {
	eMBMasterReqErrCode errorCode = MB_MRE_NO_ERR;
	uint16_t errorCount = 0;

//	traceTASK_SWITCHED_IN()
//	;

	for (;;) {
		osDelay(1);
		CpuUsageMajor = osGetCPUUsage();
		if (CpuUsageMajor > CpuUsageMinor) {
			CpuUsageMinor = CpuUsageMajor;
		}
		usSRegHoldBuf[S_HD_CPU_USAGE_MAJOR] = CpuUsageMajor;
		usSRegHoldBuf[S_HD_CPU_USAGE_MINOR] = CpuUsageMinor;
//		LED_LED1_ON;
//		LED_LED2_ON;
//		rt_thread_delay(DELAY_SYS_RUN_LED);
//		LED_LED1_OFF;
//		LED_LED2_OFF;
//		rt_thread_delay(DELAY_SYS_RUN_LED);
//		IWDG_Feed(); //feed the dog
		//Test Modbus Master
		usModbusUserData[0] = (USHORT) (HAL_GetTick() / 10);
		usModbusUserData[1] = (USHORT) (HAL_GetTick() % 10);
		ucModbusUserData[0] = 0x1F;
//		errorCode = eMBMasterReqReadDiscreteInputs(1,3,8,osWaitForever);
//		errorCode = eMBMasterReqWriteMultipleCoils(1,3,5,ucModbusUserData,osWaitForever);
//		errorCode = eMBMasterReqWriteCoil(1,8,0xFF00,osWaitForever);
//		errorCode = eMBMasterReqReadCoils(1,3,8,osWaitForever);
		errorCode = eMBMasterReqReadInputRegister(1, 3, 8, osWaitForever);
//		errorCode = eMBMasterReqWriteHoldingRegister(1,3,usModbusUserData[0],osWaitForever);
//		errorCode = eMBMasterReqWriteMultipleHoldingRegister(1,3,2,usModbusUserData,osWaitForever);
//		errorCode = eMBMasterReqReadHoldingRegister(1,3,2,osWaitForever);
//		errorCode = eMBMasterReqReadWriteMultipleHoldingRegister(1,3,2,usModbusUserData,5,2,osWaitForever);

		if (errorCode != MB_MRE_NO_ERR) {
			errorCount++;
		}
	}
}
#endif
