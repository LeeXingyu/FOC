#include "as5047p.h"

static void AS5047P_CsLow(const AS5047P_Handle_t *has5047p)
{
    HAL_GPIO_WritePin(has5047p->cs_port, has5047p->cs_pin, GPIO_PIN_RESET);
}

static void AS5047P_CsHigh(const AS5047P_Handle_t *has5047p)
{
    HAL_GPIO_WritePin(has5047p->cs_port, has5047p->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef AS5047P_ApplySpiConfig(AS5047P_Handle_t *has5047p)
{
    if ((has5047p == NULL) || (has5047p->hspi == NULL))
    {
        return HAL_ERROR;
    }

    has5047p->hspi->Init.Mode = SPI_MODE_MASTER;
    has5047p->hspi->Init.Direction = SPI_DIRECTION_2LINES;
    has5047p->hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    has5047p->hspi->Init.CLKPolarity = AS5047P_SPI_MODE_CPOL;
    has5047p->hspi->Init.CLKPhase = AS5047P_SPI_MODE_CPHA;
    has5047p->hspi->Init.NSS = SPI_NSS_SOFT;
    has5047p->hspi->Init.BaudRatePrescaler =
        (has5047p->baudrate_prescaler == 0U) ? AS5047P_SPI_DEFAULT_PRESCALER : has5047p->baudrate_prescaler;
    has5047p->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    has5047p->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    has5047p->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    has5047p->hspi->Init.CRCPolynomial = 7;
    has5047p->hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    has5047p->hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    return HAL_SPI_Init(has5047p->hspi);
}

static HAL_StatusTypeDef AS5047P_ApplyDmaConfig(AS5047P_Handle_t *has5047p)
{
    if ((has5047p == NULL) || (has5047p->hspi == NULL) ||
        (has5047p->hspi->hdmarx == NULL) || (has5047p->hspi->hdmatx == NULL))
    {
        return HAL_ERROR;
    }

    has5047p->hspi->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    has5047p->hspi->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(has5047p->hspi->hdmarx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    has5047p->hspi->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    has5047p->hspi->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(has5047p->hspi->hdmatx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef AS5047P_Init(AS5047P_Handle_t *has5047p)
{
    if ((has5047p == NULL) || (has5047p->hspi == NULL) || (has5047p->cs_port == NULL))
    {
        return HAL_ERROR;
    }

    if (AS5047P_ApplySpiConfig(has5047p) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (AS5047P_ApplyDmaConfig(has5047p) != HAL_OK)
    {
        return HAL_ERROR;
    }

    has5047p->busy = 0U;
    has5047p->tx_frame = 0xFFFFU;
    has5047p->rx_frame = 0U;
    has5047p->last_raw = 0U;
    has5047p->last_angle_deg = 0.0f;
    AS5047P_CsHigh(has5047p);
    return HAL_OK;
}

HAL_StatusTypeDef AS5047P_StartAngleReadDma(AS5047P_Handle_t *has5047p)
{
    HAL_StatusTypeDef ret;

    if ((has5047p == NULL) || (has5047p->hspi == NULL))
    {
        return HAL_ERROR;
    }

    if (has5047p->busy != 0U)
    {
        return HAL_BUSY;
    }

    has5047p->tx_frame = 0xFFFFU;
    has5047p->busy = 1U;
    AS5047P_CsLow(has5047p);
    ret = HAL_SPI_TransmitReceive_DMA(has5047p->hspi,
                                      (uint8_t *)&has5047p->tx_frame,
                                      (uint8_t *)&has5047p->rx_frame,
                                      1U);
    if (ret != HAL_OK)
    {
        AS5047P_CsHigh(has5047p);
        has5047p->busy = 0U;
    }

    return ret;
}

HAL_StatusTypeDef AS5047P_SpiDmaCpltCallback(AS5047P_Handle_t *has5047p, SPI_HandleTypeDef *hspi)
{
    uint16_t raw;

    if ((has5047p == NULL) || (hspi == NULL) || (has5047p->hspi != hspi))
    {
        return HAL_ERROR;
    }

    AS5047P_CsHigh(has5047p);
    has5047p->busy = 0U;

    raw = (uint16_t)(has5047p->rx_frame & 0x3FFFU);
    has5047p->last_raw = raw;
    has5047p->last_angle_deg = AS5047P_RawToDegrees(raw);

    return HAL_OK;
}

float AS5047P_RawToDegrees(uint16_t raw)
{
    return ((float)(raw & 0x3FFFU) * 360.0f) / 16383.0f;
}
