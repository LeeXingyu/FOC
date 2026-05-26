/*
 * drv8353rs.c
 *
 *  Created on: Apr 10, 2026
 *      Author: Administrator
 */

#include "drv8353rs.h"


extern SPI_HandleTypeDef hspi2;

DRV8353RSConfig_t drvConfig;

/**
  * 初始化配置
  */
void DRV8353RS_Init()
{
	// 使能
	DRV8353_ENABLE();

	drvConfig.drvCtrlObj.ctrlRegObj.OCP_ACT = 0b00;
	drvConfig.drvCtrlObj.ctrlRegObj.DIS_GDUV = 0b00;
	drvConfig.drvCtrlObj.ctrlRegObj.DIS_GDF = 0b00;
	drvConfig.drvCtrlObj.ctrlRegObj.OTW_REP = 0b00;
	drvConfig.drvCtrlObj.ctrlRegObj.PWM_MODE = 0b00;
	drvConfig.drvCtrlObj.ctrlRegObj.COAST = 0b00;
	drvConfig.drvCtrlObj.ctrlRegObj.BRAKE = 0b00;
	Write_Register(0x02, drvConfig.drvCtrlObj.data);

	HAL_Delay(50);

	uint16_t uData;
	HAL_StatusTypeDef ret = Read_Register(0x02, &uData);

//	uint16_t data = uData & 0x07FF;

	//printf("ret: %d, 解析: 0x%03X\r\n", ret, uData);

	HAL_Delay(100);

	drvConfig.drvCsaObj.csaObj.SEN_LVL = 0b11;
	drvConfig.drvCsaObj.csaObj.CSA_CAL_C = 0b00;
	drvConfig.drvCsaObj.csaObj.CSA_CAL_B = 0b00;
	drvConfig.drvCsaObj.csaObj.CSA_CAL_A = 0b00;
	drvConfig.drvCsaObj.csaObj.DIS_SEN = 0b00;
	drvConfig.drvCsaObj.csaObj.CSA_GAIN = 0b11;
	drvConfig.drvCsaObj.csaObj.LS_REF = 0b00;
	drvConfig.drvCsaObj.csaObj.VREF_DIV = 0b01;
	drvConfig.drvCsaObj.csaObj.CSA_FET = 0b00;

	Write_Register(0x06, drvConfig.drvCsaObj.data);

	ret = Read_Register(0x06, &uData);

	//printf("ret: %d, 解析: 0x%03X\r\n", ret, uData);
}

/**
  * 读取错误
  */
void Read_Fault(uint16_t *pData)
{
	HAL_StatusTypeDef status;
	uint16_t uTxWord = 0x8000 | ((0x00 & 0x0F) << 11);
	uint16_t uRxWord;

	DRV8353_CS_LOW();
	status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&uTxWord, (uint8_t*)&uRxWord, 1, 100);
	DRV8353_CS_HIGH();

	*pData = uRxWord & 0x07FF;

	return status;
}

/**
  * 读取寄存器数据
  */
HAL_StatusTypeDef Read_Register(uint8_t iRegAddr, uint16_t *pData)
{
	HAL_StatusTypeDef status;
	uint16_t tx_word = 0x8000 | ((iRegAddr & 0x0F) << 11);

	DRV8353_CS_LOW();
	status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&tx_word, (uint8_t*)pData, 1, 100);
	DRV8353_CS_HIGH();

	*pData = (*pData) & 0x07FF;

	return status;
}

/**
  * 写入寄存器数据
  */
HAL_StatusTypeDef Write_Register(uint8_t iRegAddr, uint16_t uData)
{
	HAL_StatusTypeDef status;
	uint16_t tx_word = 0x0000 | ((iRegAddr & 0x0F) << 11) | (uData & 0x07FF);
	uint16_t rx_word = 0;

	DRV8353_CS_LOW();
	status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&tx_word, (uint8_t*)&rx_word, 1, 100);
	DRV8353_CS_HIGH();

	return status;
}
