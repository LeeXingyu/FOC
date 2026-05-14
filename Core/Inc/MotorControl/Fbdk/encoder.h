/*
 * encoder.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 *
 *  @brief 获取编码器的原始数据、位置
 */

#ifndef INC_MOTORCONTROL_FBDK_ENCODER_H_
#define INC_MOTORCONTROL_FBDK_ENCODER_H_

#include <stdio.h>
#include <stdint.h>

#define KTH7824
//#define AS5047P

#ifdef KTH7824
#define ENCODER_COUNT 65536
#elif AS5047P
#define ENCODER_COUNT 16384
#endif

#define IDLE 0
#define SENDING 1

extern uint8_t g_spiState;

extern uint16_t g_spiTxFrame;          // SPI 发送缓冲区
extern uint16_t g_spiRxFrame;          // SPI 接收缓冲区

/**
 * @brief  初始化编码器
 */
void Init_Encoder();

/**
 * @brief  开始读取编码器数据
 */
void Start_Encoder_Read(uint8_t num);

/**
 * @brief  获取编码器原始值
 */
uint16_t Get_Encoder_Raw(uint8_t num);

/**
 * @brief  获取编码器角度
 */
uint8_t Get_Encoder_Angle(uint8_t num);

/**
 * @brief  获取编码器spi片选拉高、拉低
 */
void Encoder_CS_Low_1(uint8_t num);
void Encoder_CS_High_1(uint8_t num);


#endif /* INC_MOTORCONTROL_FBDK_ENCODER_H_ */
