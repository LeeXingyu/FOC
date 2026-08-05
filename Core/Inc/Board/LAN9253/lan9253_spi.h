/*
 * lan9252_spi.h
 *
 *  Created on: Jun 1, 2026
 *      Author: Administrator
 */

#ifndef INC_BOARD_LAN9252_LAN9252_SPI_H_
#define INC_BOARD_LAN9252_LAN9252_SPI_H_

#include "stm32g4xx_hal.h"

typedef union
{
	uint32_t Val;
	uint8_t v[4];
	uint16_t w[2];
	struct
	{
		uint8_t LB;
		uint8_t HB;
		uint8_t UB;
		uint8_t MB;
	}byte;
}UINT32_VAL;


typedef union
{
	uint16_t Val;
	struct
	{
		uint8_t LB;
		uint8_t HB;
	}byte;
}UINT16_VAL;

uint8_t WR_CMD (uint8_t cmd);
void Mem_Test(void);
void ADC_GPIO_Configuration(void);
void ADC_Configuration(void);
void NVIC_Configuration(void);
void TIM_Configuration(uint8_t period)	;
void EXTI0_Configuration(void);
void EXTI1_Configuration(void);
void EXTI8_Configuration(void);
void SPIReadDRegister(uint8_t *ReadBuffer, uint16_t Address, uint16_t Count);
void SPIWriteRegister( uint8_t *WriteBuffer, uint16_t Address, uint16_t Count);
uint32_t SPIReadDWord (uint16_t Address);
void SPIWriteDWord (uint16_t Address, uint32_t Val);

#endif /* INC_BOARD_LAN9252_LAN9252_SPI_H_ */
