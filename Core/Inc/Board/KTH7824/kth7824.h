#ifndef INC_BOARD_KTH7824_KTH7824_H_
#define INC_BOARD_KTH7824_KTH7824_H_

#include <stdint.h>
#include "main.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KTH7824_SPI_MODE_CPOL          SPI_POLARITY_HIGH
#define KTH7824_SPI_MODE_CPHA          SPI_PHASE_2EDGE
#define KTH7824_SPI_DEFAULT_PRESCALER  SPI_BAUDRATEPRESCALER_32
#define KTH7824_ANGLE_COUNTS           65536UL

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
} KTH7824_Handle_t;

HAL_StatusTypeDef KTH7824_Init(KTH7824_Handle_t *hkth7824);
HAL_StatusTypeDef KTH7824_StartAngleReadDma(KTH7824_Handle_t *hkth7824);
HAL_StatusTypeDef KTH7824_SpiDmaCpltCallback(KTH7824_Handle_t *hkth7824, SPI_HandleTypeDef *hspi);
float KTH7824_RawToDegrees(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* INC_BOARD_KTH7824_KTH7824_H_ */
