#include "kth7824.h"

static void KTH7824_CsLow(const KTH7824_Handle_t *hkth7824)
{
    HAL_GPIO_WritePin(hkth7824->cs_port, hkth7824->cs_pin, GPIO_PIN_RESET);
}

static void KTH7824_CsHigh(const KTH7824_Handle_t *hkth7824)
{
    HAL_GPIO_WritePin(hkth7824->cs_port, hkth7824->cs_pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef KTH7824_ApplySpiConfig(KTH7824_Handle_t *hkth7824)
{
    if ((hkth7824 == NULL) || (hkth7824->hspi == NULL))
    {
        return HAL_ERROR;
    }

    hkth7824->hspi->Init.Mode = SPI_MODE_MASTER;
    hkth7824->hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hkth7824->hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    hkth7824->hspi->Init.CLKPolarity = KTH7824_SPI_MODE_CPOL;
    hkth7824->hspi->Init.CLKPhase = KTH7824_SPI_MODE_CPHA;
    hkth7824->hspi->Init.NSS = SPI_NSS_SOFT;
    hkth7824->hspi->Init.BaudRatePrescaler =
        (hkth7824->baudrate_prescaler == 0U) ? KTH7824_SPI_DEFAULT_PRESCALER : hkth7824->baudrate_prescaler;
    hkth7824->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hkth7824->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hkth7824->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hkth7824->hspi->Init.CRCPolynomial = 7;
    hkth7824->hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hkth7824->hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    return HAL_SPI_Init(hkth7824->hspi);
}

static HAL_StatusTypeDef KTH7824_ApplyDmaConfig(KTH7824_Handle_t *hkth7824)
{
    if ((hkth7824 == NULL) || (hkth7824->hspi == NULL) ||
        (hkth7824->hspi->hdmarx == NULL) || (hkth7824->hspi->hdmatx == NULL))
    {
        return HAL_ERROR;
    }

    hkth7824->hspi->hdmarx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hkth7824->hspi->hdmarx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(hkth7824->hspi->hdmarx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    hkth7824->hspi->hdmatx->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hkth7824->hspi->hdmatx->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    if (HAL_DMA_Init(hkth7824->hspi->hdmatx) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef KTH7824_Init(KTH7824_Handle_t *hkth7824)
{
    if ((hkth7824 == NULL) || (hkth7824->hspi == NULL) || (hkth7824->cs_port == NULL))
    {
        return HAL_ERROR;
    }

    if (KTH7824_ApplySpiConfig(hkth7824) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (KTH7824_ApplyDmaConfig(hkth7824) != HAL_OK)
    {
        return HAL_ERROR;
    }

    hkth7824->busy = 0U;
    hkth7824->tx_frame = 0x0000U;
    hkth7824->rx_frame = 0U;
    hkth7824->last_raw = 0U;
    hkth7824->last_angle_deg = 0.0f;
    KTH7824_CsHigh(hkth7824);
    return HAL_OK;
}

HAL_StatusTypeDef KTH7824_StartAngleReadDma(KTH7824_Handle_t *hkth7824)
{
    HAL_StatusTypeDef ret;

    if ((hkth7824 == NULL) || (hkth7824->hspi == NULL))
    {
        return HAL_ERROR;
    }

    if (hkth7824->busy != 0U)
    {
        return HAL_BUSY;
    }

    hkth7824->tx_frame = 0x0000U;
    hkth7824->busy = 1U;
    KTH7824_CsLow(hkth7824);
    ret = HAL_SPI_TransmitReceive_DMA(hkth7824->hspi,
                                      (uint8_t *)&hkth7824->tx_frame,
                                      (uint8_t *)&hkth7824->rx_frame,
                                      1U);
    if (ret != HAL_OK)
    {
        KTH7824_CsHigh(hkth7824);
        hkth7824->busy = 0U;
    }

    return ret;
}

HAL_StatusTypeDef KTH7824_SpiDmaCpltCallback(KTH7824_Handle_t *hkth7824, SPI_HandleTypeDef *hspi)
{
    if ((hkth7824 == NULL) || (hspi == NULL) || (hkth7824->hspi != hspi))
    {
        return HAL_ERROR;
    }

    KTH7824_CsHigh(hkth7824);
    hkth7824->busy = 0U;
    hkth7824->last_raw = hkth7824->rx_frame;
    hkth7824->last_angle_deg = KTH7824_RawToDegrees(hkth7824->last_raw);

    return HAL_OK;
}

float KTH7824_RawToDegrees(uint16_t raw)
{
    return ((float)raw * 360.0f) / 65535.0f;
}
