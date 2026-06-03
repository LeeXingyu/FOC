#ifndef INC_BOARD_MT6835_MT6835_H_
#define INC_BOARD_MT6835_MT6835_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MT6835_SPI_MODE_CPOL              SPI_POLARITY_HIGH
#define MT6835_SPI_MODE_CPHA              SPI_PHASE_2EDGE
#define MT6835_SPI_DEFAULT_PRESCALER      SPI_BAUDRATEPRESCALER_32

#define MT6835_REG_USER_ID                0x001U
#define MT6835_REG_ANGLE_H                0x003U
#define MT6835_REG_ANGLE_M                0x004U
#define MT6835_REG_ANGLE_L                0x005U
#define MT6835_REG_ANGLE_CRC              0x006U
#define MT6835_REG_ABZ_RES_H              0x007U
#define MT6835_REG_ABZ_RES_L              0x008U
#define MT6835_REG_ZERO_POS_H             0x009U
#define MT6835_REG_ZERO_POS_L             0x00AU

#define MT6835_CMD_READ_REG               0x3U
#define MT6835_CMD_WRITE_REG              0x6U
#define MT6835_CMD_BURN_EEPROM            0xCU
#define MT6835_CMD_SET_ZERO               0x5U
#define MT6835_CMD_BURST_ANGLE            0xAU

#define MT6835_BURN_ACK                   0x55U
#define MT6835_ZERO_ACK                   0x55U

#define MT6835_STATUS_OVERSPEED           0x01U
#define MT6835_STATUS_WEAKMAG             0x02U
#define MT6835_STATUS_UNDERVOLT           0x04U

#define MT6835_ANGLE_COUNTS               2097152UL

typedef struct
{
    uint32_t angle_raw21;
    uint8_t status;
    uint8_t crc_rx;
    uint8_t crc_calc;
    float angle_deg;
} MT6835_AngleFrame_t;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t baudrate_prescaler;
    bool verify_crc;
    volatile uint8_t busy;
    uint8_t tx_buf[6];
    uint8_t rx_buf[6];
    MT6835_AngleFrame_t last_frame;
} MT6835_Handle_t;

HAL_StatusTypeDef MT6835_Init(MT6835_Handle_t *hmt);
HAL_StatusTypeDef MT6835_ReadRegister(MT6835_Handle_t *hmt, uint16_t reg_addr, uint8_t *p_value);
HAL_StatusTypeDef MT6835_WriteRegister(MT6835_Handle_t *hmt, uint16_t reg_addr, uint8_t value);
HAL_StatusTypeDef MT6835_ReadAngleBurst(MT6835_Handle_t *hmt, MT6835_AngleFrame_t *p_frame);
HAL_StatusTypeDef MT6835_StartAngleReadDma(MT6835_Handle_t *hmt);
HAL_StatusTypeDef MT6835_SpiDmaCpltCallback(MT6835_Handle_t *hmt, SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef MT6835_SetZeroCurrentPosition(MT6835_Handle_t *hmt, uint8_t *p_ack);
HAL_StatusTypeDef MT6835_BurnEeprom(MT6835_Handle_t *hmt, uint8_t *p_ack);
uint8_t MT6835_CalcCrc24(const uint8_t data_bytes[3]);
float MT6835_RawToDegrees(uint32_t raw21);

#ifdef __cplusplus
}
#endif

#endif /* INC_BOARD_MT6835_MT6835_H_ */
