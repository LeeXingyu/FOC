/* 参考示例：SPI2在 DRV8353 / MCP2518FD / LAN9253 之间切换 */
/* 说明：
 * 1) 这里只演示配置切换和CS管理思路
 * 2) 请按你的系统时钟确认 Prescaler，保证不超器件SCK上限
 */

#include "spi_switch.h"

static SPI2_Device_t s_spi2_curr_dev = (SPI2_Device_t)0xFF;

static void SPI2_AllCsHigh(void)
{
    /* 先释放所有可能占用SPI2的从设备 */
    HAL_GPIO_WritePin(DRV_CS_N_GPIO_Port, DRV_CS_N_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(COMM_CS_N_GPIO_Port, COMM_CS_N_Pin, GPIO_PIN_SET);
    /* 如果LAN9253有独立CS，也在这里拉高 */
    // HAL_GPIO_WritePin(LAN_CS_GPIO_Port, LAN_CS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef SPI2_SwitchDevice(SPI2_Device_t dev)
{
    if (s_spi2_curr_dev == dev)
    {
        return HAL_OK;
    }

    SPI2_AllCsHigh();
    __HAL_SPI_DISABLE(&hspi2);

    /* 公共项 */
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 7;
    hspi2.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    switch (dev)
    {
    case SPI2_DEV_DRV8353:
        /* DRV8353: 16-bit, Mode1(CPOL=0, CPHA=1) */
        hspi2.Init.DataSize          = SPI_DATASIZE_16BIT;
        hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        break;

    case SPI2_DEV_MCP2518FD:
        /* MCP2518FD: 8-bit, 常用Mode0(CPOL=0, CPHA=1EDGE) */
        hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
        hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        break;

    case SPI2_DEV_LAN9253:
        /* LAN9253 SPI-PDI: 8-bit, 常用Mode0 */
        hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
        hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
        hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        break;

    default:
        return HAL_ERROR;
    }

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_spi2_curr_dev = dev;
    return HAL_OK;
}
