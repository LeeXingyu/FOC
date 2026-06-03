#ifndef INC_BOARD_AS5047P_AS5047P_H_
#define INC_BOARD_AS5047P_AS5047P_H_

#include <stdint.h>
#include "main.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AS5047P_SPI_MODE_CPOL          SPI_POLARITY_LOW
#define AS5047P_SPI_MODE_CPHA          SPI_PHASE_2EDGE
#define AS5047P_SPI_DEFAULT_PRESCALER  SPI_BAUDRATEPRESCALER_32
#define AS5047P_ANGLE_COUNTS           16384UL

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t baudrate_prescaler;
    volatile uint8_t busy;
    uint16_t tx_frame;
    uint16_t rx_frame;
    uint16_t last_raw;
    float last_angle_deg;
} AS5047P_Handle_t;

HAL_StatusTypeDef AS5047P_Init(AS5047P_Handle_t *has5047p);
HAL_StatusTypeDef AS5047P_StartAngleReadDma(AS5047P_Handle_t *has5047p);
HAL_StatusTypeDef AS5047P_SpiDmaCpltCallback(AS5047P_Handle_t *has5047p, SPI_HandleTypeDef *hspi);
float AS5047P_RawToDegrees(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* INC_BOARD_AS5047P_AS5047P_H_ */
