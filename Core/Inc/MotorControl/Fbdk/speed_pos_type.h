#ifndef INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_
#define INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_

#include <stdint.h>

typedef struct
{
    union
    {
        uint32_t uAngleRawNative;
        uint32_t uAngleRaw;
    };
    float fAngle;
    float fOffsetAngle;
    union
    {
        uint32_t uOffsetAngleRawNative;
        uint32_t uOffsetAngleRaw;
    };
    float fSpeed;
    float fSpeedPll;
    float fSpeedKalman;
    uint16_t uCircle;
} SpeedAngleParam_t;

#endif /* INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_ */
