#ifndef    USER_APP
#define USER_APP
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbconfig.h"
#include "mbframe.h"
#include "mbutils.h"
#include "eeprom-board.h"

/* -----------------------Slave Defines -------------------------------------*/
#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   16
#define S_COIL_START                  0
#define S_COIL_NCOILS                 64
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             10
#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           10
/* salve mode: holding register's all address */
#define          S_HD_RESERVE                     0
#define          S_HD_CPU_USAGE_MAJOR             1
#define          S_HD_CPU_USAGE_MINOR             2
/* salve mode: input register's all address */
#define          S_IN_RESERVE                     0
/* salve mode: coil's all address */
#define          S_CO_RESERVE                     0
/* salve mode: discrete's all address */
#define          S_DI_RESERVE                     0

/* -----------------------Master Defines -------------------------------------*/
#define M_DISCRETE_INPUT_START        0
#define M_DISCRETE_INPUT_NDISCRETES   8//16
#define M_COIL_START                  0
#define M_COIL_NCOILS                 8//64
#define M_REG_INPUT_START             0
#define M_REG_INPUT_NREGS             10//100
#define M_REG_HOLDING_START           0
#define M_REG_HOLDING_NREGS           5//100
/* master mode: holding register's all address */
#define          M_HD_RESERVE                     0
/* master mode: input register's all address */
#define          M_IN_RESERVE                     0
/* master mode: coil's all address */
#define          M_CO_RESERVE                     0
/* master mode: discrete's all address */
#define          M_DI_RESERVE                     0
extern USHORT   usMRegInBuf[][M_REG_INPUT_NREGS];
extern eMBMasterEventType eQueuedMasterEvent;

typedef struct {
  uint16_t slave_id;
  uint16_t reg_addr;
  uint16_t qty;
  uint16_t baudrate;
  uint16_t interval;
} setting_t;

extern setting_t user_setting;

#define USER_SETTING_EEPROM_BASE 0x100

uint8_t eMBMasterValidUserSetting(uint16_t reg, const uint8_t * setting_data);
void eMBMasterWriteUserSetting(const uint8_t * setting_data);
            
#endif
