/*
 * drv8353rs.h
 *
 *  Created on: Apr 10, 2026
 *      Author: Administrator
 */


#ifndef INC_BOARD_DRV8353RS_DRV8353RS_H_
#define INC_BOARD_DRV8353RS_DRV8353RS_H_

#include <stdio.h>
#include <stdint.h>
#include "drv8353rs_type.h"

void DRV8353RS_Init(void);

/* 修正：返回 HAL_StatusTypeDef，便于上层判断 */
HAL_StatusTypeDef Read_Fault(uint16_t *pData);

/* 新增：刷新故障寄存器到全局 drvConfig */
HAL_StatusTypeDef DRV8353_UpdateFaultStatus(void);
HAL_StatusTypeDef DRV8353_ClearFault(void);
HAL_StatusTypeDef DRV8353_SelfTest(uint16_t *pCtrlReg, uint16_t *pCsaReg);

/* 原有接口 */
HAL_StatusTypeDef Read_Register(uint8_t iRegAddr, uint16_t *pData);
HAL_StatusTypeDef Write_Register(uint8_t iRegAddr, uint16_t uData);

#endif /* INC_BOARD_DRV8353RS_DRV8353RS_H_ */
