/*
 * drv8353rs_type.h
 *
 *  Created on: Apr 10, 2026
 *      Author: Administrator
 */

#include "stm32g4xx_hal.h"

#ifndef INC_BOARD_DRV8353RS_DRV8353RS_TYPE_H_
#define INC_BOARD_DRV8353RS_DRV8353RS_TYPE_H_

/* GPIO 定义 */
#define DRV8353_CS_PORT         GPIOD
#define DRV8353_CS_PIN          GPIO_PIN_2
#define DRV8353_ENABLE_PORT     GPIOB
#define DRV8353_ENABLE_PIN      GPIO_PIN_6


/* 片选控制宏 */
#define DRV8353_CS_LOW()        HAL_GPIO_WritePin(DRV8353_CS_PORT, DRV8353_CS_PIN, GPIO_PIN_RESET)
#define DRV8353_CS_HIGH()       HAL_GPIO_WritePin(DRV8353_CS_PORT, DRV8353_CS_PIN, GPIO_PIN_SET)
#define DRV8353_ENABLE()        HAL_GPIO_WritePin(DRV8353_ENABLE_PORT, DRV8353_ENABLE_PIN, GPIO_PIN_SET)
#define DRV8353_DISABLE()       HAL_GPIO_WritePin(DRV8353_ENABLE_PORT, DRV8353_ENABLE_PIN, GPIO_PIN_RESET)


/*
    故障状态寄存器1 (address = 0x00h)
*/
typedef struct
{
    uint16_t VDS_LC    : 1;
    uint16_t VDS_HC    : 1;
    uint16_t VDS_LB    : 1;
    uint16_t VDS_HB    : 1;

    uint16_t VDS_LA    : 1;
    uint16_t VDS_HA    : 1;
    uint16_t OTSD      : 1;
    uint16_t UVLO      : 1;

    uint16_t GDF       : 1;
    uint16_t VDS_OCP   : 1;
    uint16_t FAULT     : 1;

    uint16_t res       : 5;

} FaultStatusReg1bit_t;

typedef struct
{
   union
   {
      uint16_t data;
      FaultStatusReg1bit_t fault1RegObj;
   };
} FaultStatusReg1_t;

/*
    故障状态寄存器2 (address = 0x01h)
*/
typedef struct
{
    uint16_t VGS_LC    : 1;
    uint16_t VGS_HC    : 1;

    uint16_t VGS_LB    : 1;
    uint16_t VGS_HB    : 1;

    uint16_t VDS_LA    : 1;
    uint16_t VDS_HA    : 1;

    uint16_t GDUV      : 1;
    uint16_t OTW       : 1;

    uint16_t SC_OC     : 1;
    uint16_t SB_OC     : 1;
    uint16_t SA_OC     : 1;

    uint16_t res       : 5;

} FaultStatusReg2bit_t;

typedef struct
{
   union
   {
      uint16_t data;
      FaultStatusReg2bit_t fault2RegObj;
   };
} FaultStatusReg2_t;

/*
    驱动控制寄存器 (address = 0x02h)
*/
typedef struct
{
    uint16_t CLR_FLT  : 1;
    uint16_t BRAKE    : 1;
    uint16_t COAST    : 1;
    uint16_t PWM1_DIR : 1;

    uint16_t PWM1_COM : 1;
    uint16_t PWM_MODE : 2;
    uint16_t OTW_REP  : 1;

    uint16_t DIS_GDF  : 1;
    uint16_t DIS_GDUV : 1;
    uint16_t OCP_ACT  : 1;

    uint16_t res      : 5;

} DrvCtrlRegbit_t;

typedef struct
{
   union
   {
      uint16_t data;
      DrvCtrlRegbit_t ctrlRegObj;
   };
} DrvCtrlReg_t;

/*
    Gate Drive HS Register (address = 0x03h)
*/
typedef struct
{
    uint16_t IDRIVEN_HS  : 4;
    uint16_t IDRIVEP_HS  : 4;

    uint16_t LOCK        : 3;
    uint16_t res         : 5;

}DrvGateHSbit_t;

typedef struct
{
   union
   {
      uint16_t data;
      DrvGateHSbit_t gateHSRegObj;
   };
} DrvGateHS_t;


/*
    Gate Drive LS Register (address = 0x04h)
*/
typedef struct
{
    uint16_t IDRIVEN_LS  : 4;
    uint16_t IDRIVEP_LS  : 4;

    uint16_t TDRIVE      : 2;
    uint16_t CBC         : 1;

    uint16_t res         : 5;

}DrvGateLSbit_t;

typedef struct
{
   union
   {
      uint16_t data;
      DrvGateLSbit_t gateLSRegObj;
   };
} DrvGateLS_t;

/*
    OCP Control Register (address = 0x05h)
*/
typedef struct
{
    uint16_t VDS_LVL     : 4;
    uint16_t OCP_DEG     : 2;

    uint16_t OCP_MODE    : 2;
    uint16_t DEAD_TIME   : 2;

    uint16_t TRETRY      : 1;

    uint16_t res         : 5;

} DrvOCPbit_t;

typedef struct
{
   union
   {
      uint16_t data;
      DrvOCPbit_t ocpObj;
   };
} DrvOCP_t;

/*
    CSA Control Register (DRV8353 and DRV8353R Only) (address = 0x06h)
*/
typedef struct
{
    uint16_t SEN_LVL     : 2;

    uint16_t CSA_CAL_C   : 1;
    uint16_t CSA_CAL_B   : 1;
    uint16_t CSA_CAL_A   : 1;
    uint16_t DIS_SEN     : 1;

    uint16_t CSA_GAIN    : 2;
    uint16_t LS_REF      : 1;
    uint16_t VREF_DIV    : 1;

    uint16_t CSA_FET     : 1;
    uint16_t res         : 5;

} DrvCSAbit_t;

typedef struct
{
   union
   {
      uint16_t data;
      DrvCSAbit_t csaObj;
   };
} DrvCSA_t;

/*
    Driver Configuration Register (DRV8353 and DRV8353R Only) (address = 0x07h)
*/
typedef struct
{
    uint16_t SEN_LVL     : 1;
    uint16_t RES         : 10;
    uint16_t res         : 5;

} DrvCfgbit_t;

typedef struct
{
   union
   {
      uint16_t data;
      DrvCfgbit_t cfgObj;
   };
} DrvCfg_t;


typedef struct
{
    DrvCfg_t drvCfgObj;
    DrvCSA_t drvCsaObj;
    DrvOCP_t drvOcpObj;

    DrvGateLS_t drvGateLSObj;
    DrvGateHS_t drvGateHSObj;

    FaultStatusReg1_t  faultStatusReg1Obj;
    FaultStatusReg2_t  faultStatusReg2Obj;

    DrvCtrlReg_t drvCtrlObj;
} DRV8353RSConfig_t;


#endif /* INC_BOARD_DRV8353RS_DRV8353RS_TYPE_H_ */
