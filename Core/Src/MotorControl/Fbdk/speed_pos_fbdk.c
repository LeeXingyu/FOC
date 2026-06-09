#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "speed_pos_type.h"
#include "speed_pos_fbdk.h"
#include "encoder.h"
#include "main.h"
#include "mc_math.h"
#include "mc_interface.h"
#include "motor_parameters.h"

static uint32_t s_prevCircleRawNative = 0U;
static uint32_t s_prevSpeedRawNative = 0U;
static int64_t s_absEncoderNative = 0;
static bool s_circleHistoryValid = false;
static bool s_speedHistoryValid = false;

static uint32_t SpeedPos_GetNativeCounts(void)
{
    return Get_Angle_CountNative();
}

static int32_t SpeedPos_SignedDelta(uint32_t cur, uint32_t prev, uint32_t counts)
{
    uint32_t direct;
    uint32_t wrap;

    if (counts <= 1U)
    {
        return 0;
    }

    if (cur >= prev)
    {
        direct = cur - prev;
        wrap = counts - direct;
        return (direct <= wrap) ? (int32_t)direct : -(int32_t)wrap;
    }

    direct = prev - cur;
    wrap = counts - direct;
    return (direct <= wrap) ? -(int32_t)direct : (int32_t)wrap;
}

static int64_t SpeedPos_FloorDiv(int64_t numerator, int64_t denominator)
{
    int64_t q;
    int64_t r;

    if (denominator == 0)
    {
        return 0;
    }

    q = numerator / denominator;
    r = numerator % denominator;
    if ((r != 0) && (((r < 0) && (denominator > 0)) || ((r > 0) && (denominator < 0))))
    {
        q -= 1;
    }

    return q;
}

static void SpeedPos_ResetCircleHistory(uint32_t rawNative)
{
    s_absEncoderNative = (int64_t)rawNative;
    s_prevCircleRawNative = rawNative;
    s_circleHistoryValid = true;
}

static void SpeedPos_ResetSpeedHistory(uint32_t rawNative)
{
    s_prevSpeedRawNative = rawNative;
    s_speedHistoryValid = true;
}

void Circle_Update(void)
{
    uint32_t counts;
    uint32_t rawNative;
    int32_t delta;
    int64_t zeroAdjusted;

    counts = SpeedPos_GetNativeCounts();
    rawNative = Get_Angle_RawNative();

    if (!g_axis.posCtrl.bCalibFlag || (counts <= 1U))
    {
        SpeedPos_ResetCircleHistory(rawNative);
        g_axis.fbdk.uCircle = 0U;
        return;
    }

    if (!s_circleHistoryValid)
    {
        SpeedPos_ResetCircleHistory(rawNative);
        g_axis.fbdk.uCircle = 0U;
        return;
    }

    delta = SpeedPos_SignedDelta(rawNative, s_prevCircleRawNative, counts);
    s_absEncoderNative += (int64_t)delta;
    s_prevCircleRawNative = rawNative;

    if (g_axis.posCtrl.uOffsetAngleRawNative < counts)
    {
        zeroAdjusted = s_absEncoderNative - (int64_t)g_axis.posCtrl.uOffsetAngleRawNative;
        if (zeroAdjusted >= 0)
        {
            g_axis.fbdk.uCircle = (uint16_t)SpeedPos_FloorDiv(zeroAdjusted, (int64_t)counts);
        }
        else
        {
            g_axis.fbdk.uCircle = 0U;
        }
    }
    else
    {
        g_axis.fbdk.uCircle = 0U;
    }
}

void Sensor_Update_Kalman(void)
{
    uint32_t curRawNative;
    uint32_t counts;
    int32_t deltaNative;
    float speedRpmRaw = 0.0f;
    const float samplePeriodS = 1.0f / SPEED_MEASUREMENT_RATE_HZ;

    curRawNative = Get_Angle_RawNative();
    counts = SpeedPos_GetNativeCounts();

    if (!g_axis.posCtrl.bCalibFlag || (counts <= 1U))
    {
        g_axis.fbdk.fSpeedKalman = 0.0f;
        SpeedPos_ResetCircleHistory(curRawNative);
        SpeedPos_ResetSpeedHistory(curRawNative);
        return;
    }

    if (!s_speedHistoryValid)
    {
        SpeedPos_ResetSpeedHistory(curRawNative);
        g_axis.fbdk.fSpeedKalman = 0.0f;
        return;
    }

    deltaNative = SpeedPos_SignedDelta(curRawNative, s_prevSpeedRawNative, counts);
    if (fabsf((float)deltaNative) > 1.0f)
    {
        speedRpmRaw = ((float)deltaNative / (float)counts) / samplePeriodS * 60.0f;
    }

    g_axis.fbdk.fSpeedKalman = Kalman_Filter_Calc(&g_motorSpeedKalmanFilter, speedRpmRaw);
    s_prevSpeedRawNative = curRawNative;
}

void Calc_Speed(fixp30_t *pSpeed)
{
    uint32_t rawNative;
    float fElectricalFreqHz;

    if (pSpeed == NULL)
    {
        return;
    }

    rawNative = Get_Angle_RawNative();

    if (!g_axis.posCtrl.bCalibFlag)
    {
        g_axis.fbdk.fSpeedKalman = 0.0f;
        SpeedPos_ResetCircleHistory(rawNative);
        SpeedPos_ResetSpeedHistory(rawNative);
        *pSpeed = FIXP30(0.0f);
        return;
    }

    Circle_Update();
    Sensor_Update_Kalman();

    fElectricalFreqHz = ((float)MC_Get_Pole_Pairs() * g_axis.fbdk.fSpeedKalman) / 60.0f;
    *pSpeed = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
}

void Get_Speed(fixp30_t *pSpeed)
{
    float fElectricalFreqHz;

    if (pSpeed == NULL)
    {
        return;
    }

    fElectricalFreqHz = ((float)MC_Get_Pole_Pairs() * g_axis.fbdk.fSpeedKalman) / 60.0f;
    *pSpeed = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
}

void Get_Angle(fixp30_t *pAngle)
{
    uint32_t curRaw;
    uint32_t sumCount;
    uint32_t tempRaw;
    float fElecAngleUnwrapped;
    float fMechAngle;

    if (pAngle == NULL)
    {
        return;
    }

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

uint16_t Get_Angle_Raw(void)
{
    return g_axis.fbdk.uAngleRaw;
}

uint32_t Get_Angle_RawNative(void)
{
    return Get_Encoder_RawNative(ENC_ID_MOTOR);
}

uint32_t Get_Angle_CountNative(void)
{
    return Get_Encoder_NativeCount(ENC_ID_MOTOR);
}

void Sensor_Update(void)
{
}

void Sensor_Update_PLL(void)
{
}
