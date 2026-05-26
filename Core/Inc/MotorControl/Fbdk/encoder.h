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
#include "spi.h"
#include "main.h"

//#define AS5047P

#ifdef KTH7824
#define ENCODER_COUNT 65536
#elif AS5047P
#define ENCODER_COUNT 16384
#endif

typedef enum
{
    ENC_ID_MOTOR = 0,   // 绑定SPI1
    ENC_ID_LOAD = 1,   // 绑定SPI3
    ENC_ID_MAX
} EncoderId_t;

typedef enum
{
    ENC_SPI_IDLE = 0,
    ENC_SPI_BUSY
} EncoderSpiState_t;

void Init_Encoder(void);
HAL_StatusTypeDef Start_Encoder_Read(EncoderId_t id);
uint16_t Get_Encoder_Raw(EncoderId_t id);
float Get_Encoder_AngleDeg(EncoderId_t id);

/* 在HAL_SPI_TxRxCpltCallback里调用 */
void Encoder_SpiDmaCpltCallback(SPI_HandleTypeDef *hspi);

#endif /* INC_MOTORCONTROL_FBDK_ENCODER_H_ */



