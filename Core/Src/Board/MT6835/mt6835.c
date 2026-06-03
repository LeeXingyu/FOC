#include "mt6835.h"

static void MT6835_CsLow(const MT6835_Handle_t *hmt)
{
    HAL_GPIO_WritePin(hmt->cs_port, hmt->cs_pin, GPIO_PIN_RESET);
}

static void MT6835_CsHigh(const MT6835_Handle_t *hmt)
{
    HAL_GPIO_WritePin(hmt->cs_port, hmt->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MT6835_ApplySpiConfig(MT6835_Handle_t *hmt)
{
    if ((hmt == NULL) || (hmt->hspi == NULL))
    {
        return HAL_ERROR;
    }

    hmt->hspi->Init.Mode = SPI_MODE_MASTER;
    hmt->hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hmt->hspi->Init.DataSize = SPI_DATASIZE_8BIT;
    hmt->hspi->Init.CLKPolarity = MT6835_SPI_MODE_CPOL;
    hmt->hspi->Init.CLKPhase = MT6835_SPI_MODE_CPHA;
    hmt->hspi->Init.NSS = SPI_NSS_SOFT;
    hmt->hspi->Init.BaudRatePrescaler =
        (hmt->baudrate_prescaler == 0U) ? MT6835_SPI_DEFAULT_PRESCALER : hmt->baudrate_prescaler;
    hmt->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hmt->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hmt->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hmt->hspi->Init.CRCPolynomial = 7;
    hmt->hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hmt->hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    return HAL_SPI_Init(hmt->hspi);
}

static HAL_StatusTypeDef MT6835_ApplyDmaConfig(MT6835_Handle_t *hmt)
{
    if ((hmt == NULL) || (hmt->hspi == NULL) || (hmt->hspi->hdmarx == NULL) || (hmt->hspi->hdmatx == NULL))
    {
        return HAL_ERROR;
    }

    hmt->hspi->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hmt->hspi->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    if (HAL_DMA_Init(hmt->hspi->hdmarx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    hmt->hspi->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hmt->hspi->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    if (HAL_DMA_Init(hmt->hspi->hdmatx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void MT6835_DecodeFrame(MT6835_Handle_t *hmt, MT6835_AngleFrame_t *p_frame, const uint8_t *rx)
{
    uint8_t crc_data[3];

    p_frame->angle_raw21 = (((uint32_t)rx[2]) << 13)
                         | (((uint32_t)rx[3]) << 5)
                         | (((uint32_t)rx[4]) >> 3);
    p_frame->status = (uint8_t)(rx[4] & 0x07U);
    p_frame->crc_rx = rx[5];
    crc_data[0] = rx[2];
    crc_data[1] = rx[3];
    crc_data[2] = rx[4];
    p_frame->crc_calc = MT6835_CalcCrc24(crc_data);
    p_frame->angle_deg = MT6835_RawToDegrees(p_frame->angle_raw21);

    hmt->last_frame = *p_frame;
}

static HAL_StatusTypeDef MT6835_Transfer(const MT6835_Handle_t *hmt,
                                         const uint8_t *p_tx,
                                         uint8_t *p_rx,
                                         uint16_t size)
{
    HAL_StatusTypeDef st;

    if ((hmt == NULL) || (hmt->hspi == NULL) || (p_tx == NULL) || (p_rx == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    MT6835_CsLow(hmt);
    st = HAL_SPI_TransmitReceive(hmt->hspi, (uint8_t *)p_tx, p_rx, size, 100U);
    MT6835_CsHigh(hmt);

    return st;
}

HAL_StatusTypeDef MT6835_Init(MT6835_Handle_t *hmt)
{
    HAL_StatusTypeDef st;

    if ((hmt == NULL) || (hmt->hspi == NULL) || (hmt->cs_port == NULL))
    {
        return HAL_ERROR;
    }

    st = MT6835_ApplySpiConfig(hmt);
    if (st != HAL_OK)
    {
        return st;
    }

    st = MT6835_ApplyDmaConfig(hmt);
    if (st != HAL_OK)
    {
        return st;
    }

    MT6835_CsHigh(hmt);
    hmt->busy = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef MT6835_ReadRegister(MT6835_Handle_t *hmt, uint16_t reg_addr, uint8_t *p_value)
{
    uint8_t tx[3];
    uint8_t rx[3];
    HAL_StatusTypeDef st;

    if ((hmt == NULL) || (p_value == NULL))
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)((MT6835_CMD_READ_REG << 4) | ((reg_addr >> 8) & 0x0FU));
    tx[1] = (uint8_t)(reg_addr & 0xFFU);
    tx[2] = 0x00U;

    st = MT6835_Transfer(hmt, tx, rx, 3U);
    if (st != HAL_OK)
    {
        return st;
    }

    *p_value = rx[2];
    return HAL_OK;
}

HAL_StatusTypeDef MT6835_WriteRegister(MT6835_Handle_t *hmt, uint16_t reg_addr, uint8_t value)
{
    uint8_t tx[3];
    uint8_t rx[3];

    if (hmt == NULL)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)((MT6835_CMD_WRITE_REG << 4) | ((reg_addr >> 8) & 0x0FU));
    tx[1] = (uint8_t)(reg_addr & 0xFFU);
    tx[2] = value;

    return MT6835_Transfer(hmt, tx, rx, 3U);
}

HAL_StatusTypeDef MT6835_ReadAngleBurst(MT6835_Handle_t *hmt, MT6835_AngleFrame_t *p_frame)
{
    uint8_t tx[6];
    uint8_t rx[6];
    HAL_StatusTypeDef st;

    if ((hmt == NULL) || (p_frame == NULL))
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)((MT6835_CMD_BURST_ANGLE << 4) | ((MT6835_REG_ANGLE_H >> 8) & 0x0FU));
    tx[1] = (uint8_t)(MT6835_REG_ANGLE_H & 0xFFU);
    tx[2] = 0x00U;
    tx[3] = 0x00U;
    tx[4] = 0x00U;
    tx[5] = 0x00U;

    st = MT6835_Transfer(hmt, tx, rx, 6U);
    if (st != HAL_OK)
    {
        return st;
    }

    MT6835_DecodeFrame(hmt, p_frame, rx);

    if (hmt->verify_crc && (p_frame->crc_calc != p_frame->crc_rx))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MT6835_StartAngleReadDma(MT6835_Handle_t *hmt)
{
    HAL_StatusTypeDef st;

    if ((hmt == NULL) || (hmt->hspi == NULL))
    {
        return HAL_ERROR;
    }

    if (hmt->busy != 0U)
    {
        return HAL_BUSY;
    }

    hmt->tx_buf[0] = (uint8_t)((MT6835_CMD_BURST_ANGLE << 4) | ((MT6835_REG_ANGLE_H >> 8) & 0x0FU));
    hmt->tx_buf[1] = (uint8_t)(MT6835_REG_ANGLE_H & 0xFFU);
    hmt->tx_buf[2] = 0x00U;
    hmt->tx_buf[3] = 0x00U;
    hmt->tx_buf[4] = 0x00U;
    hmt->tx_buf[5] = 0x00U;

    hmt->busy = 1U;
    MT6835_CsLow(hmt);
    st = HAL_SPI_TransmitReceive_DMA(hmt->hspi, hmt->tx_buf, hmt->rx_buf, 6U);
    if (st != HAL_OK)
    {
        MT6835_CsHigh(hmt);
        hmt->busy = 0U;
    }

    return st;
}

HAL_StatusTypeDef MT6835_SpiDmaCpltCallback(MT6835_Handle_t *hmt, SPI_HandleTypeDef *hspi)
{
    if ((hmt == NULL) || (hspi == NULL) || (hmt->hspi != hspi))
    {
        return HAL_ERROR;
    }

    MT6835_CsHigh(hmt);
    hmt->busy = 0U;
    MT6835_DecodeFrame(hmt, &hmt->last_frame, hmt->rx_buf);

    if (hmt->verify_crc && (hmt->last_frame.crc_calc != hmt->last_frame.crc_rx))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MT6835_SetZeroCurrentPosition(MT6835_Handle_t *hmt, uint8_t *p_ack)
{
    uint8_t tx[3];
    uint8_t rx[3];
    HAL_StatusTypeDef st;

    if (hmt == NULL)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(MT6835_CMD_SET_ZERO << 4);
    tx[1] = 0x00U;
    tx[2] = 0x00U;

    st = MT6835_Transfer(hmt, tx, rx, 3U);
    if ((st == HAL_OK) && (p_ack != NULL))
    {
        *p_ack = rx[2];
    }

    return st;
}

HAL_StatusTypeDef MT6835_BurnEeprom(MT6835_Handle_t *hmt, uint8_t *p_ack)
{
    uint8_t tx[3];
    uint8_t rx[3];
    HAL_StatusTypeDef st;

    if (hmt == NULL)
    {
        return HAL_ERROR;
    }

    tx[0] = (uint8_t)(MT6835_CMD_BURN_EEPROM << 4);
    tx[1] = 0x00U;
    tx[2] = 0x00U;

    st = MT6835_Transfer(hmt, tx, rx, 3U);
    if ((st == HAL_OK) && (p_ack != NULL))
    {
        *p_ack = rx[2];
    }

    return st;
}

uint8_t MT6835_CalcCrc24(const uint8_t data_bytes[3])
{
    uint8_t crc = 0x00U;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 3U; i++)
    {
        crc ^= data_bytes[i];
        for (j = 0U; j < 8U; j++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

float MT6835_RawToDegrees(uint32_t raw21)
{
    return ((float)(raw21 & 0x1FFFFFU) * 360.0f) / (float)MT6835_ANGLE_COUNTS;
}
