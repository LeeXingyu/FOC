/*
 * drv8353rs.c
 *
 *  Created on: Apr 10, 2026
 *      Author: Administrator
 */


#include "drv8353rs.h"
extern SPI_HandleTypeDef hspi2;

/* 保持你的全局命名 */
DRV8353RSConfig_t drvConfig;

void DRV8353RS_Init(void)
{
    DRV8353_ENABLE();

    drvConfig.drvCtrlObj.ctrlRegObj.OCP_ACT   = 0b00;
    drvConfig.drvCtrlObj.ctrlRegObj.DIS_GDUV  = 0b00;
    drvConfig.drvCtrlObj.ctrlRegObj.DIS_GDF   = 0b00;
    drvConfig.drvCtrlObj.ctrlRegObj.OTW_REP   = 0b00;
    drvConfig.drvCtrlObj.ctrlRegObj.PWM_MODE  = 0b00;
    drvConfig.drvCtrlObj.ctrlRegObj.COAST     = 0b00;
    drvConfig.drvCtrlObj.ctrlRegObj.BRAKE     = 0b00;
    (void)Write_Register(0x02, drvConfig.drvCtrlObj.data);

    HAL_Delay(50);
    (void)Read_Register(0x02, &drvConfig.drvCtrlObj.data);

    HAL_Delay(100);

    drvConfig.drvCsaObj.csaObj.SEN_LVL    = 0b11;
    drvConfig.drvCsaObj.csaObj.CSA_CAL_C  = 0b00;
    drvConfig.drvCsaObj.csaObj.CSA_CAL_B  = 0b00;
    drvConfig.drvCsaObj.csaObj.CSA_CAL_A  = 0b00;
    drvConfig.drvCsaObj.csaObj.DIS_SEN    = 0b00;
    drvConfig.drvCsaObj.csaObj.CSA_GAIN   = 0b11;
    drvConfig.drvCsaObj.csaObj.LS_REF     = 0b00;
    drvConfig.drvCsaObj.csaObj.VREF_DIV   = 0b01;
    drvConfig.drvCsaObj.csaObj.CSA_FET    = 0b00;

    (void)Write_Register(0x06, drvConfig.drvCsaObj.data);
    (void)Read_Register(0x06, &drvConfig.drvCsaObj.data);
}

/* 仅读 0x00：兼容你现有调用 */
HAL_StatusTypeDef Read_Fault(uint16_t *pData)
{
    if (pData == NULL) return HAL_ERROR;
    return Read_Register(0x00, pData);
}

/* 读取 0x00 和 0x01 并写入 drvConfig 的 fault 结构体 */
HAL_StatusTypeDef DRV8353_UpdateFaultStatus(void)
{
    HAL_StatusTypeDef st;
    uint16_t reg = 0U;

    st = Read_Register(0x00, &reg);
    if (st != HAL_OK) return st;
    drvConfig.faultStatusReg1Obj.data = reg & 0x07FFU;

    st = Read_Register(0x01, &reg);
    if (st != HAL_OK) return st;
    drvConfig.faultStatusReg2Obj.data = reg & 0x07FFU;

    return HAL_OK;
}


HAL_StatusTypeDef Read_Register(uint8_t iRegAddr, uint16_t *pData)
{
    HAL_StatusTypeDef status;
    uint16_t tx_word;

    if (pData == NULL) return HAL_ERROR;

    tx_word = 0x8000U | (((uint16_t)iRegAddr & 0x0FU) << 11);

    DRV8353_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&tx_word, (uint8_t *)pData, 1, 100);
    DRV8353_CS_HIGH();

    *pData &= 0x07FFU;
    return status;
}

HAL_StatusTypeDef Write_Register(uint8_t iRegAddr, uint16_t uData)
{
    HAL_StatusTypeDef status;
    uint16_t tx_word = (((uint16_t)iRegAddr & 0x0FU) << 11) | (uData & 0x07FFU);
    uint16_t rx_word = 0U;

    DRV8353_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&tx_word, (uint8_t *)&rx_word, 1, 100);
    DRV8353_CS_HIGH();

    return status;
}

HAL_StatusTypeDef DRV8353_ClearFault(void)
{
    uint16_t ctrl = 0U;
    HAL_StatusTypeDef st = Read_Register(0x02, &ctrl);
    if (st != HAL_OK) return st;

    ctrl |= 0x0001U;                /* CLR_FLT */
    st = Write_Register(0x02, ctrl);
    if (st != HAL_OK) return st;

    ctrl &= (uint16_t)(~0x0001U);   /* 可选：手动拉回 */
    return Write_Register(0x02, ctrl);
}