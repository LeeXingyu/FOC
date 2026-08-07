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

// Legacy project command ID layout:
// [10:4] function code, [3:0] node ID.
#define CAN_NODE_ID_BITS             4U
#define CAN_NODE_ID_MASK             0x0FU
#define CAN_FUNCTION_CODE_SHIFT       CAN_NODE_ID_BITS
#define CAN_MAKE_ID(func, node)      (uint16_t)((((uint16_t)(func)) << CAN_FUNCTION_CODE_SHIFT) | ((uint16_t)(node) & CAN_NODE_ID_MASK))
#define CAN_GET_FUNC(id)             (uint8_t)((uint16_t)(id) >> CAN_FUNCTION_CODE_SHIFT)
#define CAN_GET_NODE(id)             (uint8_t)((uint16_t)(id) & CAN_NODE_ID_MASK)

/* Standard CANopen COB-IDs for the single-axis CiA 402 node. */
#define CANOPEN_COBID_NMT             0x000U
#define CANOPEN_COBID_TPDO1_BASE      0x180U
#define CANOPEN_COBID_RPDO1_BASE      0x200U
#define CANOPEN_COBID_TPDO2_BASE      0x280U
#define CANOPEN_COBID_RPDO2_BASE      0x300U
#define CANOPEN_COBID_TPDO3_BASE      0x380U
#define CANOPEN_COBID_RPDO3_BASE      0x400U
#define CANOPEN_COBID_TPDO4_BASE      0x480U
#define CANOPEN_COBID_RPDO4_BASE      0x500U
#define CANOPEN_COBID_SDO_TX_BASE     0x580U
#define CANOPEN_COBID_SDO_RX_BASE     0x600U
#define CANOPEN_COBID_HEARTBEAT_BASE  0x700U
#define CANOPEN_NODE_ID_MIN            1U
#define CANOPEN_NODE_ID_MAX            127U

#ifndef APP_USE_CAN_FD
#define APP_USE_CAN_FD               0
#endif

/*
 * Keep standard CANopen/CiA 402 traffic isolated by default.  The legacy
 * function-code command and telemetry transport remains available for
 * compatibility builds by defining this switch to 1.
 */
#ifndef APP_USE_LEGACY_CAN_PROTOCOL
#define APP_USE_LEGACY_CAN_PROTOCOL   0U
#endif

#if APP_USE_CAN_FD
#define APP_CAN_TX_FIFO_PAYLOAD_SIZE CAN_PLSIZE_16
#define APP_CAN_RX_FIFO_PAYLOAD_SIZE CAN_PLSIZE_16
#define APP_CAN_FRAME_FDF            1U
#define APP_CAN_FRAME_BRS            1U
#define APP_CAN_MAX_DATA_BYTES       16U
#define APP_CAN_RX_FETCH_BYTES       16U
#define APP_CAN_BITTIME_SETUP        CAN_500K_2M
#else
#define APP_CAN_TX_FIFO_PAYLOAD_SIZE CAN_PLSIZE_16
#define APP_CAN_RX_FIFO_PAYLOAD_SIZE CAN_PLSIZE_16
#define APP_CAN_FRAME_FDF            0U
#define APP_CAN_FRAME_BRS            0U
#define APP_CAN_MAX_DATA_BYTES       8U
#define APP_CAN_RX_FETCH_BYTES       8U
#define APP_CAN_BITTIME_SETUP        CAN_500K_2M
#endif

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
    uint32_t tx_timeout_count;
    uint32_t tx_drop_count;
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
