#ifndef _SPI_SWITCH_H
#define _SPI_SWITCH_H

#include "spi.h"
#include "main.h"

typedef enum
{
    SPI2_DEV_DRV8353 = 0,
    SPI2_DEV_MCP2518FD,
    SPI2_DEV_LAN9253
} SPI2_Device_t;

HAL_StatusTypeDef SPI2_SwitchDevice(SPI2_Device_t dev);

#endif
