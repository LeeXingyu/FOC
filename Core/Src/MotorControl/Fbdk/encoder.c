/*
 * encoder.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */
#include "encoder.h"
#include "speed_pos_type.h"
#include "stm32g4xx_hal.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;


uint8_t g_spiState = IDLE;
uint16_t g_spiTxFrame = 0;          // SPI 发送缓冲区
uint16_t g_spiRxFrame = 0;          // SPI 接收缓冲区

volatile int32_t s_iAngleAcc = 0;       // 供速度计算用的累积计数
static uint16_t  s_uAccPrevRaw = 0;		// 编码器上一次的读取数据
static uint8_t   s_uAccInited   = 0;		// 读取逻辑是否初始化（过滤第一次处理）

/**
 * @brief  初始化编码器
 */
void Init_Encoder()
{
	g_axis.fbdk.fAngle = 0.0f;
	g_axis.fbdk.uAngleRaw = 0;
	g_axis.fbdk.fSpeed = 0.0f;
	g_axis.fbdk.fSpeedKalman = 0.0f;
	g_axis.fbdk.fSpeedPll = 0.0f;
	g_axis.fbdk.fOffsetAngle = 0.0f;
	g_axis.fbdk.uOffsetAngleRaw = 0;


#ifdef KTH7824

#elif AS5047P

#endif
}

/**
 * @brief  获取编码器spi片选拉低(选择)
 */
void Encoder_CS_Low(uint8_t num)
{
	if(num == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	else if(num == 2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	}

}


/**
 * @brief  获取编码器spi片选拉高
 */
void Encoder_CS_High(uint8_t num)
{
	if(num == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else if(num == 2)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	}
}


/**
 * @brief  开始读取编码器数据
 */
void Start_Encoder_Read(uint8_t num)
{
	if (g_spiState == IDLE)
	{
		g_spiState = SENDING;

		Encoder_CS_Low(num);

#ifdef KTH7824
		// 启动第一次 DMA 传输（发送读取命令）
		g_spiTxFrame = 0x00;
		HAL_SPI_TransmitReceive_DMA(&hspi3, &g_spiTxFrame,
									&g_spiRxFrame, 1);
#elif AS5047P

#endif

	}
}

/**
 * @brief  获取编码器原始值
 */
uint16_t Get_Encoder_Raw(uint8_t num)
{

}

/**
 * @brief  获取编码器角度
 */
uint8_t Get_Encoder_Angle(uint8_t num)
{

}

/**
 * @brief  编码器spi+dma回调
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		if (g_spiState == SENDING)
		{
#ifdef KTH7824
			g_spiTxFrame = 0x00;
			HAL_SPI_TransmitReceive_DMA(&hspi3, &g_spiTxFrame, &g_spiRxFrame, 1);

			g_axis.fbdk.uAngleRaw = g_spiRxFrame;

			uint16_t uRaw = g_axis.fbdk.uAngleRaw;

			if (!s_uAccInited) {
				s_uAccPrevRaw = uRaw;
				s_uAccInited   = 1;
			} else {
				int32_t iDelta = (int32_t)uRaw - (int32_t)s_iAngleAcc;
				if (iDelta >  32768) iDelta -= 65536;
				if (iDelta < -32768) iDelta += 65536;
				s_iAngleAcc   += iDelta;
				s_uAccPrevRaw = uRaw;
			}

			g_axis.fbdk.fAngle = (float)uRaw * (360.0f / 65535.0f);

			Encoder_CS_High(1);
			Encoder_CS_High(2);

			g_spiState = IDLE;

#elif AS5047P

#endif
		}

	}
}



