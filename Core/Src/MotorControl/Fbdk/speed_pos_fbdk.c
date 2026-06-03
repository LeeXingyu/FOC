#include <math.h>

#include "speed_pos_type.h"
#include "speed_pos_fbdk.h"
#include "encoder.h"
#include "main.h"
#include "mc_math.h"
#include "mc_interface.h"
#include "motor_parameters.h"

static uint32_t s_prevEncoderRawNative = 0U;

void Circle_Update()
{
}

void Calc_Speed(fixp30_t *pSpeed)
{
    static uint8_t sfirstSample = 1U;
    float fElectricalFreqHz;

    if (!sfirstSample)
    {
        Sensor_Update_Kalman();
    }
    else
    {
        sfirstSample = 0U;
    }

    fElectricalFreqHz = ((float)MC_Get_Pole_Pairs() * g_axis.fbdk.fSpeedKalman) / 60.0f;
    *pSpeed = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
}

void Get_Speed(fixp30_t *pSpeed)
{
    float fElectricalFreqHz = ((float)MC_Get_Pole_Pairs() * g_axis.fbdk.fSpeedKalman) / 60.0f;
    *pSpeed = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
}

void Get_Angle(fixp30_t *pAngle)
{
    uint32_t curRaw;
    uint32_t sumCount;
    uint32_t tempRaw;
    float fElecAngleUnwrapped;
    float fMechAngle;

    if (!g_axis.posCtrl.bCalibFlag)
    {
        *pAngle = FIXP30(0.0f);
        return;
    }

    curRaw = Get_Angle_RawNative();
    sumCount = Get_Angle_CountNative();
    if (sumCount <= 1U)
    {
        *pAngle = FIXP30(0.0f);
        return;
    }

    if (curRaw >= g_axis.posCtrl.uOffsetAngleRawNative)
    {
        tempRaw = curRaw - g_axis.posCtrl.uOffsetAngleRawNative;
    }
    else
    {
        tempRaw = curRaw + sumCount - g_axis.posCtrl.uOffsetAngleRawNative;
    }

    fElecAngleUnwrapped = (float)tempRaw / (float)sumCount;
    fMechAngle = fElecAngleUnwrapped * (float)MC_Get_Pole_Pairs();
    *pAngle = FIXP30(fmodf(fMechAngle, 1.0f));
}

uint16_t Get_Angle_Raw()
{
    return g_axis.fbdk.uAngleRaw;
}

uint32_t Get_Angle_RawNative()
{
    return Get_Encoder_RawNative(ENC_ID_MOTOR);
}

uint32_t Get_Angle_CountNative()
{
    return Get_Encoder_NativeCount(ENC_ID_MOTOR);
}

void Sensor_Update()
{
}

void Sensor_Update_Kalman()
{
    uint32_t curRawNative = Get_Angle_RawNative();
    uint32_t sumCountNative = Get_Angle_CountNative();
    float fDiffRaw = 0.0f;
    float fSamplePeriod = 1.0f / 1000.0f;
    float fSpeedRpmRaw = 0.0f;

    if (sumCountNative <= 1U)
    {
        g_axis.fbdk.fSpeedKalman = 0.0f;
        s_prevEncoderRawNative = curRawNative;
        return;
    }

    if (curRawNative >= s_prevEncoderRawNative)
    {
        uint32_t diffPos = curRawNative - s_prevEncoderRawNative;
        uint32_t diffNeg = sumCountNative - diffPos;
        fDiffRaw = (diffPos <= diffNeg) ? (float)diffPos : -(float)diffNeg;
    }
    else
    {
        uint32_t diffNeg = s_prevEncoderRawNative - curRawNative;
        uint32_t diffPos = sumCountNative - diffNeg;
        fDiffRaw = (diffNeg < diffPos) ? -(float)diffNeg : (float)diffPos;
    }

    if (fabsf(fDiffRaw) <= 4.0f)
    {
        fSpeedRpmRaw = 0.0f;
    }
    else
    {
        fSpeedRpmRaw = (fDiffRaw / (float)sumCountNative) / fSamplePeriod * 60.0f;
    }

    g_axis.fbdk.fSpeedKalman = Kalman_Filter_Calc(&g_motorSpeedKalmanFilter, fSpeedRpmRaw);
    s_prevEncoderRawNative = curRawNative;
}

void Sensor_Update_PLL()
{
}
