#ifndef INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_
#define INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_

#include <stdint.h>

typedef struct
{
    uint16_t uAngleRaw;
    uint32_t uAngleRawNative;
    float fAngle;
    float fOffsetAngle;
    uint16_t uOffsetAngleRaw;
    uint32_t uOffsetAngleRawNative;
    float fSpeed;
    float fSpeedPll;
    float fSpeedKalman;
    uint16_t uCircle;
} SpeedAngleParam_t;

#endif /* INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_ */
