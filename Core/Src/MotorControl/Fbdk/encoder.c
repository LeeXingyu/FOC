#include "encoder.h"
#include "speed_pos_type.h"
#include "stm32g4xx_hal.h"


extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    volatile EncoderSpiState_t state;
    volatile uint16_t tx_frame;
    volatile uint16_t rx_frame;
    volatile uint16_t raw;
    volatile float angle_deg;
} EncoderDev_t;

static EncoderDev_t s_enc[ENC_ID_MAX] =
{
    [ENC_ID_MOTOR] = {
        .hspi = &hspi1,
        .cs_port = CODER_CS_N2_GPIO_Port,
        .cs_pin = CODER_CS_N2_Pin,
        .state = ENC_SPI_IDLE,
        .tx_frame = 0U,
        .rx_frame = 0U,
        .raw = 0U,
        .angle_deg = 0.0f
    },
    [ENC_ID_LOAD] = {
        .hspi = &hspi3,
        .cs_port = CODER_CS_N1_GPIO_Port,
        .cs_pin = CODER_CS_N1_Pin,
        .state = ENC_SPI_IDLE,
        .tx_frame = 0U,
        .rx_frame = 0U,
        .raw = 0U,
        .angle_deg = 0.0f
    }
};

static inline void Encoder_CS_Low(EncoderId_t id)
{
    HAL_GPIO_WritePin(s_enc[id].cs_port, s_enc[id].cs_pin, GPIO_PIN_RESET);
}

static inline void Encoder_CS_High(EncoderId_t id)
{
    HAL_GPIO_WritePin(s_enc[id].cs_port, s_enc[id].cs_pin, GPIO_PIN_SET);
}

void Init_Encoder(void)
{
    uint8_t i;
    g_axis.fbdk.fAngle = 0.0f;
    g_axis.fbdk.uAngleRaw = 0U;
    g_axis.fbdk.fSpeed = 0.0f;
    g_axis.fbdk.fSpeedKalman = 0.0f;
    g_axis.fbdk.fSpeedPll = 0.0f;
    g_axis.fbdk.fOffsetAngle = 0.0f;
    g_axis.fbdk.uOffsetAngleRaw = 0U;

    for (i = 0U; i < ENC_ID_MAX; i++)
    {
        s_enc[i].state = ENC_SPI_IDLE;
        s_enc[i].tx_frame = 0U;
        s_enc[i].rx_frame = 0U;
        s_enc[i].raw = 0U;
        s_enc[i].angle_deg = 0.0f;
        Encoder_CS_High((EncoderId_t)i);
    }
}

HAL_StatusTypeDef Start_Encoder_Read(EncoderId_t id)
{
    HAL_StatusTypeDef ret;

    if ((id >= ENC_ID_MAX) || (s_enc[id].state != ENC_SPI_IDLE))
    {
        return HAL_BUSY;
    }

    s_enc[id].state = ENC_SPI_BUSY;
    Encoder_CS_Low(id);

#ifdef KTH7824
    s_enc[id].tx_frame = 0x0000U;
    ret = HAL_SPI_TransmitReceive_DMA(s_enc[id].hspi,
                                      (uint16_t *)&s_enc[id].tx_frame,
                                      (uint16_t *)&s_enc[id].rx_frame,
                                      1U);
#elif defined(AS5047P)
    s_enc[id].tx_frame = 0xFFFFU;
    ret = HAL_SPI_TransmitReceive_DMA(s_enc[id].hspi,
                                      (uint16_t *)&s_enc[id].tx_frame,
                                      (uint16_t *)&s_enc[id].rx_frame,
                                      1U);
#else
    s_enc[id].tx_frame = 0x0000U;
    ret = HAL_SPI_TransmitReceive_DMA(s_enc[id].hspi,
                                      (uint16_t *)&s_enc[id].tx_frame,
                                      (uint16_t *)&s_enc[id].rx_frame,
                                      1U);
#endif

    if (ret != HAL_OK)
    {
        Encoder_CS_High(id);
        s_enc[id].state = ENC_SPI_IDLE;
    }

    return ret;
}

uint16_t Get_Encoder_Raw(EncoderId_t id)
{
    if (id >= ENC_ID_MAX)
    {
        return 0U;
    }
    return s_enc[id].raw;
}

float Get_Encoder_AngleDeg(EncoderId_t id)
{
    if (id >= ENC_ID_MAX)
    {
        return 0.0f;
    }
    return s_enc[id].angle_deg;
}

void Encoder_SpiDmaCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t i;
    for (i = 0U; i < ENC_ID_MAX; i++)
    {
        if ((s_enc[i].hspi == hspi) && (s_enc[i].state == ENC_SPI_BUSY))
        {
            uint16_t raw = s_enc[i].rx_frame;

#ifdef KTH7824
            s_enc[i].raw = raw;
            s_enc[i].angle_deg = ((float)raw) * (360.0f / 65535.0f);
#elif defined(AS5047P)
            raw &= 0x3FFFU;
            s_enc[i].raw = raw;
            s_enc[i].angle_deg = ((float)raw) * (360.0f / 16383.0f);
#else
            s_enc[i].raw = raw;
            s_enc[i].angle_deg = ((float)raw) * (360.0f / 65535.0f);
#endif

            Encoder_CS_High((EncoderId_t)i);
            s_enc[i].state = ENC_SPI_IDLE;

        if ((EncoderId_t)i == ENC_ID_MOTOR)
        {
            g_axis.fbdk.uAngleRaw = s_enc[i].raw;
            g_axis.fbdk.fAngle    = s_enc[i].angle_deg;
        }
        // else if ((EncoderId_t)i == ENC_ID_LOAD)
        // {
        //     g_loadAngleRaw  = s_enc[i].raw;
        //     g_loadAngleDeg  = s_enc[i].angle_deg;
        //     g_loadDataValid = 1U;
        // }
            return;
        }
    }
}

/* 放到你原有HAL回调里 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Encoder_SpiDmaCpltCallback(hspi);
}