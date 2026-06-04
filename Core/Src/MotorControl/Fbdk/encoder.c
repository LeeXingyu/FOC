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
    EncoderType_t type;
    uint8_t enabled;
    uint8_t fault_latched;
    uint8_t timeout_count;
    volatile EncoderSpiState_t state;
    uint32_t start_tick;
    volatile uint16_t raw_compat;
    volatile uint32_t raw_native;
    volatile float angle_deg;
    AS5047P_Handle_t as5047p;
    KTH7824_Handle_t kth7824;
    MT6835_Handle_t mt6835;
} EncoderDev_t;

static EncoderDev_t s_enc[ENC_ID_MAX] =
{
    [ENC_ID_MOTOR] = {
        .hspi = &hspi1,
        .cs_port = CODER_CS_N2_GPIO_Port,
        .cs_pin = CODER_CS_N2_Pin,
        .type = MOTOR_ENCODER_TYPE,
        .state = ENC_SPI_IDLE,
        .raw_compat = 0U,
        .raw_native = 0U,
        .angle_deg = 0.0f,
        .as5047p = {
            .hspi = &hspi1,
            .cs_port = CODER_CS_N2_GPIO_Port,
            .cs_pin = CODER_CS_N2_Pin,
            .baudrate_prescaler = SPI_BAUDRATEPRESCALER_32
        },
        .kth7824 = {
            .hspi = &hspi1,
            .cs_port = CODER_CS_N2_GPIO_Port,
            .cs_pin = CODER_CS_N2_Pin,
            .baudrate_prescaler = SPI_BAUDRATEPRESCALER_32
        },
        .mt6835 = {
            .hspi = &hspi1,
            .cs_port = CODER_CS_N2_GPIO_Port,
            .cs_pin = CODER_CS_N2_Pin,
            .baudrate_prescaler = SPI_BAUDRATEPRESCALER_32,
            .verify_crc = false
        }
    },
    [ENC_ID_LOAD] = {
        .hspi = &hspi3,
        .cs_port = CODER_CS_N1_GPIO_Port,
        .cs_pin = CODER_CS_N1_Pin,
        .type = LOAD_ENCODER_TYPE,
        .state = ENC_SPI_IDLE,
        .raw_compat = 0U,
        .raw_native = 0U,
        .angle_deg = 0.0f,
        .as5047p = {
            .hspi = &hspi3,
            .cs_port = CODER_CS_N1_GPIO_Port,
            .cs_pin = CODER_CS_N1_Pin,
            .baudrate_prescaler = SPI_BAUDRATEPRESCALER_32
        },
        .kth7824 = {
            .hspi = &hspi3,
            .cs_port = CODER_CS_N1_GPIO_Port,
            .cs_pin = CODER_CS_N1_Pin,
            .baudrate_prescaler = SPI_BAUDRATEPRESCALER_32
        },
        .mt6835 = {
            .hspi = &hspi3,
            .cs_port = CODER_CS_N1_GPIO_Port,
            .cs_pin = CODER_CS_N1_Pin,
            .baudrate_prescaler = SPI_BAUDRATEPRESCALER_32,
            .verify_crc = false
        }
    }
};

#define ENCODER_SPI_TIMEOUT_MS  5U
#define ENCODER_MAX_TIMEOUT_COUNT  3U

static uint32_t Encoder_CountFromType(EncoderType_t type)
{
    switch (type)
    {
        case ENC_TYPE_AS5047P:
            return AS5047P_ANGLE_COUNTS;

        case ENC_TYPE_MT6835:
            return MT6835_ANGLE_COUNTS;

        case ENC_TYPE_KTH7824:
        default:
            return KTH7824_ANGLE_COUNTS;
    }
}

static uint16_t Encoder_NormalizeToCompat16(uint32_t raw_native, uint32_t native_counts)
{
    if (native_counts <= 1U)
    {
        return 0U;
    }

    raw_native %= native_counts;
    return (uint16_t)(((uint64_t)raw_native * 65535ULL) / (uint64_t)(native_counts - 1U));
}

static HAL_StatusTypeDef Encoder_InitDevice(EncoderDev_t *pdev)
{
    switch (pdev->type)
    {
        case ENC_TYPE_AS5047P:
            return AS5047P_Init(&pdev->as5047p);

        case ENC_TYPE_MT6835:
            return MT6835_Init(&pdev->mt6835);

        case ENC_TYPE_KTH7824:
        default:
            return KTH7824_Init(&pdev->kth7824);
    }
}

static HAL_StatusTypeDef Encoder_StartRead(EncoderDev_t *pdev)
{
    switch (pdev->type)
    {
        case ENC_TYPE_AS5047P:
            return AS5047P_StartAngleReadDma(&pdev->as5047p);

        case ENC_TYPE_MT6835:
            return MT6835_StartAngleReadDma(&pdev->mt6835);

        case ENC_TYPE_KTH7824:
        default:
            return KTH7824_StartAngleReadDma(&pdev->kth7824);
    }
}

static HAL_StatusTypeDef Encoder_CompleteRead(EncoderDev_t *pdev, SPI_HandleTypeDef *hspi)
{
    HAL_StatusTypeDef st;

    switch (pdev->type)
    {
        case ENC_TYPE_AS5047P:
            st = AS5047P_SpiDmaCpltCallback(&pdev->as5047p, hspi);
            if (st == HAL_OK)
            {
                pdev->raw_native = pdev->as5047p.last_raw;
                pdev->angle_deg = pdev->as5047p.last_angle_deg;
            }
            return st;

        case ENC_TYPE_MT6835:
            st = MT6835_SpiDmaCpltCallback(&pdev->mt6835, hspi);
            if (st == HAL_OK)
            {
                pdev->raw_native = pdev->mt6835.last_frame.angle_raw21;
                pdev->angle_deg = pdev->mt6835.last_frame.angle_deg;
            }
            return st;

        case ENC_TYPE_KTH7824:
        default:
            st = KTH7824_SpiDmaCpltCallback(&pdev->kth7824, hspi);
            if (st == HAL_OK)
            {
                pdev->raw_native = pdev->kth7824.last_raw;
                pdev->angle_deg = pdev->kth7824.last_angle_deg;
            }
            return st;
    }
}

static void Encoder_UpdateCompatFeedback(EncoderDev_t *pdev)
{
    pdev->raw_compat = Encoder_NormalizeToCompat16(pdev->raw_native, Encoder_CountFromType(pdev->type));
}

static void Encoder_DisableDevice(EncoderDev_t *pdev)
{
    if (pdev == NULL)
    {
        return;
    }

    pdev->enabled = 0U;
    pdev->fault_latched = 1U;
    pdev->state = ENC_SPI_IDLE;
    pdev->start_tick = 0U;
}

static void Encoder_ForceStop(EncoderDev_t *pdev)
{
    if ((pdev == NULL) || (pdev->hspi == NULL))
    {
        return;
    }

    (void)HAL_SPI_DMAStop(pdev->hspi);
    HAL_GPIO_WritePin(pdev->cs_port, pdev->cs_pin, GPIO_PIN_SET);

    switch (pdev->type)
    {
        case ENC_TYPE_AS5047P:
            pdev->as5047p.busy = 0U;
            break;

        case ENC_TYPE_MT6835:
            pdev->mt6835.busy = 0U;
            break;

        case ENC_TYPE_KTH7824:
        default:
            pdev->kth7824.busy = 0U;
            break;
    }

    pdev->state = ENC_SPI_IDLE;
    pdev->start_tick = 0U;
}

static void Encoder_CheckTimeout(EncoderDev_t *pdev)
{
    uint32_t now;

    if ((pdev == NULL) || (pdev->state != ENC_SPI_BUSY))
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - pdev->start_tick) >= ENCODER_SPI_TIMEOUT_MS)
    {
        Encoder_ForceStop(pdev);
        if (pdev->timeout_count < 0xFFU)
        {
            pdev->timeout_count++;
        }
        if (pdev->timeout_count >= ENCODER_MAX_TIMEOUT_COUNT)
        {
            Encoder_DisableDevice(pdev);
        }
    }
}

void Init_Encoder(void)
{
    uint8_t i;

    g_axis.fbdk.fAngle = 0.0f;
    g_axis.fbdk.uAngleRaw = 0U;
    g_axis.fbdk.uAngleRawNative = 0U;
    g_axis.fbdk.fSpeed = 0.0f;
    g_axis.fbdk.fSpeedKalman = 0.0f;
    g_axis.fbdk.fSpeedPll = 0.0f;
    g_axis.fbdk.fOffsetAngle = 0.0f;
    g_axis.fbdk.uOffsetAngleRaw = 0U;
    g_axis.fbdk.uOffsetAngleRawNative = 0U;

    for (i = 0U; i < ENC_ID_MAX; i++)
    {
        s_enc[i].enabled = 1U;
        s_enc[i].fault_latched = 0U;
        s_enc[i].timeout_count = 0U;
        s_enc[i].state = ENC_SPI_IDLE;
        s_enc[i].start_tick = 0U;
        s_enc[i].raw_compat = 0U;
        s_enc[i].raw_native = 0U;
        s_enc[i].angle_deg = 0.0f;
        if (Encoder_InitDevice(&s_enc[i]) != HAL_OK)
        {
            Encoder_DisableDevice(&s_enc[i]);
        }
    }
}

HAL_StatusTypeDef Start_Encoder_Read(EncoderId_t id)
{
    HAL_StatusTypeDef ret;

    if (id >= ENC_ID_MAX)
    {
        return HAL_ERROR;
    }

    if (s_enc[id].enabled == 0U)
    {
        return HAL_ERROR;
    }

    Encoder_CheckTimeout(&s_enc[id]);

    if (s_enc[id].enabled == 0U)
    {
        return HAL_ERROR;
    }

    if (s_enc[id].state != ENC_SPI_IDLE)
    {
        return HAL_BUSY;
    }

    s_enc[id].state = ENC_SPI_BUSY;
    s_enc[id].start_tick = HAL_GetTick();
    ret = Encoder_StartRead(&s_enc[id]);
    if (ret != HAL_OK)
    {
        s_enc[id].state = ENC_SPI_IDLE;
        s_enc[id].start_tick = 0U;
    }

    return ret;
}

uint16_t Get_Encoder_Raw(EncoderId_t id)
{
    if (id >= ENC_ID_MAX)
    {
        return 0U;
    }

    return s_enc[id].raw_compat;
}

uint16_t Get_Encoder_RawCompat(EncoderId_t id)
{
    return Get_Encoder_Raw(id);
}

uint32_t Get_Encoder_RawNative(EncoderId_t id)
{
    if (id >= ENC_ID_MAX)
    {
        return 0U;
    }

    return s_enc[id].raw_native;
}

uint32_t Get_Encoder_NativeCount(EncoderId_t id)
{
    if (id >= ENC_ID_MAX)
    {
        return 0U;
    }

    return Encoder_CountFromType(s_enc[id].type);
}

EncoderType_t Get_Encoder_Type(EncoderId_t id)
{
    if (id >= ENC_ID_MAX)
    {
        return ENC_TYPE_KTH7824;
    }

    return s_enc[id].type;
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
            if (Encoder_CompleteRead(&s_enc[i], hspi) == HAL_OK)
            {
                s_enc[i].timeout_count = 0U;
                Encoder_UpdateCompatFeedback(&s_enc[i]);

                if ((EncoderId_t)i == ENC_ID_MOTOR)
                {
                    g_axis.fbdk.uAngleRaw = s_enc[i].raw_compat;
                    g_axis.fbdk.uAngleRawNative = s_enc[i].raw_native;
                    g_axis.fbdk.fAngle = s_enc[i].angle_deg;
                }
            }

            s_enc[i].state = ENC_SPI_IDLE;
            s_enc[i].start_tick = 0U;
            return;
        }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    Encoder_SpiDmaCpltCallback(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t i;

    for (i = 0U; i < ENC_ID_MAX; i++)
    {
        if (s_enc[i].hspi == hspi)
        {
            Encoder_ForceStop(&s_enc[i]);
            if (s_enc[i].timeout_count < 0xFFU)
            {
                s_enc[i].timeout_count++;
            }
            if (s_enc[i].timeout_count >= ENCODER_MAX_TIMEOUT_COUNT)
            {
                Encoder_DisableDevice(&s_enc[i]);
            }
            return;
        }
    }
}
