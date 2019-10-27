#include "main.h"
#include "cmsis_os.h"

#include "mb.h"
#include "mbport.h"
#include "user_mb_app.h"
#include "cpu_utils.h"
#include "util_console.h"
//#include "usbd_cdc_if.h"
#include "lora.h"
//USHORT usModbusUserData[MB_PDU_SIZE_MAX];
//UCHAR ucModbusUserData[MB_PDU_SIZE_MAX];
extern LoraFlagStatus AppProcessRequest;
//extern eMBMasterEventType eQueuedMasterEvent;
#ifdef USE_LORA_HAL_DRIVER
extern osThreadId mLoraTaskHandle;
//extern void thread_LoRaPoll(void const * argument);
#endif

__ALIGNED(portBYTE_ALIGNMENT)
#if MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0
extern USHORT usSRegInBuf[];
void thread_ModbusSlavePoll(void const * argument)
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
void thread_ModbusMasterPoll(void const * argument) {
//	traceTASK_SWITCHED_IN()
//	;
        static uint32_t baud = 4800;
        //TODO: Read EEPROM for User settings
        EepromMcuReadBuffer(USER_SETTING_EEPROM_BASE, (uint8_t *) &user_setting, sizeof (setting_t) );

        eMBMasterNormalizeUserSetting(&user_setting);

        switch(user_setting.baudrate){
          case 0:
            baud = 2400;
            break;
          case 1:
            baud = 4800;
            break;
          case 2:
            baud = 9600;
            break;
          case 3:
            baud = 19200;
          break;
          case 4:
            baud = 57600;
          break;
          case 5:
            baud = 115200;
          break;
          default:
            baud = 4800;
            break;
        };

	eMBErrorCode eStatus = eMBMasterInit(MB_RTU, user_setting.slave_id, baud, MB_PAR_NONE);

	if (eStatus == MB_ENOERR) {
		eStatus = eMBMasterEnable();
	}

	for (;;) {
		osDelay(100);
		eMBMasterPoll();
	}
}

void thread_Simulation(void const * argument) {
	eMBMasterReqErrCode errorCode = MB_MRE_NO_ERR;
	uint16_t errorCount = 0;

//	UCHAR ucSndAddr = 1;
//	USHORT usRegAddr = 0;
//	USHORT usNRegs = 6;
	uint32_t tInterval = 1000;//	TickType_t tInterval = pdMS_TO_TICKS(1000);

//	traceTASK_SWITCHED_IN()
//	;

	for (;;) {
		osDelay(user_setting.interval); //vTaskDelay(tInterval);
		CpuUsageMajor = osGetCPUUsage();
		if (CpuUsageMajor > CpuUsageMinor) {
			CpuUsageMinor = CpuUsageMajor;
		}
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
//                while (eQueuedMasterEvent != EV_MASTER_READY)
		errorCode = eMBMasterReqReadInputRegister(user_setting.slave_id,
				user_setting.reg_addr, user_setting.qty, tInterval);
		if (errorCode == MB_MRE_NO_ERR) {
			AppProcessRequest = LORA_SET;
			vTaskResume(mLoraTaskHandle);
		}
//		errorCode = eMBMasterReqWriteHoldingRegister(1,3,usModbusUserData[0],-1);
//		errorCode = eMBMasterReqWriteMultipleHoldingRegister(1,3,2,usModbusUserData,-1);
//		while (eQueuedMasterEvent != EV_MASTER_READY)
		errorCode = eMBMasterReqReadHoldingRegister(7,0,5, tInterval);
//		errorCode = eMBMasterReqReadWriteMultipleHoldingRegister(1,3,2,usModbusUserData,5,2,-1);

		if (errorCode != MB_MRE_NO_ERR) {
			errorCount++;
//                        errorCode = MB_MRE_NO_ERR;
		} 
//                else 
//                {
////			CDC_Transmit_FS((uint8_t *) &usMRegInBuf[ucSndAddr - 1][0], usNRegs);
//			PRINTF("Temp=%d\r\n", usMRegInBuf[ucSndAddr - 1][0]);
//		}

	}
}
#endif
