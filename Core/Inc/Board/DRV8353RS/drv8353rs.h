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

/**
  * 初始化配置
  */
void DRV8353RS_Init();

/**
  * 读取错误
  * @param pData 寄存器地址
  */
void Read_Fault(uint16_t *pData);

/**
  * 读取寄存器数据
  * @param iRegAddr 寄存器地址
  * @param pData 读取的数据
  */
HAL_StatusTypeDef Read_Register(uint8_t iRegAddr, uint16_t *pData);

/**
  * 读取寄存器数据
  * @param iRegAddr 寄存器地址
  * @param uData 写入的数据
  */
HAL_StatusTypeDef Write_Register(uint8_t iRegAddr, uint16_t uData);

#endif /* INC_BOARD_DRV8353RS_DRV8353RS_H_ */
