#ifndef INC_MOTORCONTROL_FBDK_ENCODER_H_
#define INC_MOTORCONTROL_FBDK_ENCODER_H_

#include <stdint.h>
#include "main.h"
#include "spi.h"
#include "as5047p.h"
#include "kth7824.h"
#include "mt6835.h"

/* FOC control uses a unified 16-bit electrical angle domain for all encoder types. */
#define ENCODER_FOC_COMPAT_COUNT  65536UL
#define ENCODER_COUNT             ENCODER_FOC_COMPAT_COUNT

typedef enum
{
    ENC_TYPE_KTH7824 = BOARD_ENCODER_TYPE_KTH7824,
    ENC_TYPE_AS5047P = BOARD_ENCODER_TYPE_AS5047P,
    ENC_TYPE_MT6835 = BOARD_ENCODER_TYPE_MT6835
} EncoderType_t;

#ifndef MOTOR_ENCODER_TYPE
#define MOTOR_ENCODER_TYPE        BOARD_ENCODER_TYPE_MT6835
#endif

#ifndef LOAD_ENCODER_TYPE
#define LOAD_ENCODER_TYPE         MOTOR_ENCODER_TYPE
#endif

typedef enum
{
    ENC_ID_MOTOR = 0,
    ENC_ID_LOAD = 1,
    ENC_ID_MAX
} EncoderId_t;

typedef enum
{
    ENC_SPI_IDLE = 0,
    ENC_SPI_BUSY
} EncoderSpiState_t;

void Init_Encoder(void);
HAL_StatusTypeDef Start_Encoder_Read(EncoderId_t id);
uint16_t Get_Encoder_Raw(EncoderId_t id);
uint32_t Get_Encoder_RawNative(EncoderId_t id);
uint32_t Get_Encoder_NativeCount(EncoderId_t id);
EncoderType_t Get_Encoder_Type(EncoderId_t id);
float Get_Encoder_AngleDeg(EncoderId_t id);
uint16_t Get_Encoder_RawCompat(EncoderId_t id);
void Encoder_SpiDmaCpltCallback(SPI_HandleTypeDef *hspi);

#endif /* INC_MOTORCONTROL_FBDK_ENCODER_H_ */
