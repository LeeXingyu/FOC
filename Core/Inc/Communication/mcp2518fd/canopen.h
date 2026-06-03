#ifndef _CANOPEN_H
#define	_CANOPEN_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "spi.h"
#include "gpio.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_register.h"
#include "stm32g4xx_hal.h"
#include "main.h"
//========================================================//
#ifdef	__cplusplus
extern "C" {
#endif

// Index to SPI channel
// Used when multiple MCP2517FD are connected to the same SPI interface, but with different CS    
#define DRV_CANFDSPI_INDEX_0         0
#define DRV_CANFDSPI_INDEX_1         1

//! SPI Initialization
    
void CANFD_INIT(void);

//! SPI Read/Write Transfer

HAL_StatusTypeDef DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t XferSize);

void MCP2518FD_TransmitMessageQueue(CANFDSPI_MODULE_ID index, uint16_t id, uint8_t *data, CAN_DLC len);
void MCP2518FD_ReceiveMessage(CANFDSPI_MODULE_ID index, uint8_t nBytes);

typedef struct
{
    uint32_t rx_irq_count;
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t rx_overflow_count;
    uint32_t rx_invalid_count;
    uint32_t rx_reject_count;
    uint16_t last_rx_sid;
    uint8_t  last_rx_len;
    uint8_t  last_rx_status;
} MCP2518FD_Status_t;

void MCP2518FD_ProcessRxIrq(void);
void MCP2518FD_Service1ms(void);
MCP2518FD_Status_t MCP2518FD_GetStatus(void);

//========================================================//
#ifdef	__cplusplus
}
#endif
//========================================================//
#endif
