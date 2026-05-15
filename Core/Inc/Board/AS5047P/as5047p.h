/*
 * as5047p.h
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */

#ifndef INC_BOARD_AS5047P_AS5047P_H_
#define INC_BOARD_AS5047P_AS5047P_H_

#include "spi.h"
#include "main.h"
#include "gpio.h"

// as5047p.h
#define AS5047P_READ_CMD      0xFFFF  // 读取命令 (发送任意值，返回角度数据)
#define AS5047P_CLEAR_ERROR   0x0001  // 清除错误标志
#define AS5047P_AGC_REG       0x3FFD  // 自动增益控制寄存器
#define AS5047P_MAG_REG       0x3FFE  // 磁场强度寄存器
#define AS5047P_ANGLE_REG     0x3FFF  // 角度寄存器 (只读)

// 角度数据结构
typedef struct {
  uint16_t raw_angle;    // 原始角度值 (0-16383)
  float angle_deg;       // 角度值 (0.0-359.9°)
  uint16_t agc_value;    // 自动增益控制值
  uint16_t mag_status;   // 磁场强度状态
  uint8_t error_flag;    // 错误标志
} AS5047P_Data;

#define CODER_CS_LOW_1()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define CODER_CS_HIGH_1()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define CODER_CS_LOW_2()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define CODER_CS_HIGH_2()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)

AS5047P_Data Read_AS5047P_Angle(uint8_t coder_num);


#endif /* INC_BOARD_AS5047P_AS5047P_H_ */
