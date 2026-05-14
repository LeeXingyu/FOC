/*
 * speed_pos_fbdk.c
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */

#include "speed_pos_type.h"
#include "speed_pos_fbdk.h"
#include "encoder.h"
#include "main.h"
#include "mc_math.h"
#include "motor_parameters.h"

// 上一时刻编码器值
float s_prevEncoderRaw;

/**
 * @brief  圈数量计算
 */
void Circle_Update()
{

}

/**
  * @brief  计算并获取电气速度
  *
  * @param  pSpeed: 电机速度
 */
void Calc_Speed(fixp30_t *pSpeed)
{
	//	if (!g_axis.bPolarityFlag)
	//	{
	//		return;
	//	}

	static uint8_t sfirstSample = 1;

	if (!sfirstSample)
	{
		Sensor_Update_Kalman();
	}
	else
	{
		sfirstSample = 0;
	}

	// 电气频率
	float fElectricalFreqHz = ((float)POLE_PAIR_NUM * g_axis.fbdk.fSpeedKalman) / 60.0f;

	// 归一化输出
	*pSpeed = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
}

/**
  * @brief  获取电气速度
  *
  * @param  pSpeed: 电机速度
 */
void Get_Speed(fixp30_t *pSpeed)
{
	float fElectricalFreqHz = ((float)POLE_PAIR_NUM * g_axis.fbdk.fSpeedKalman) / 60.0f;
	*pSpeed = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
}

/**
  * @brief  获取电气角度
  *
  * @param  pAngle: 电气角度
 */

void Get_Angle(fixp30_t *pAngle)
{
	if (!g_axis.posCtrl.bCalibFlag)
	{
		*pAngle = FIXP30(0.0);
		return;
	}
	else if (g_axis.posCtrl.uOffsetAngleRaw != 0 )
	{
		uint16_t uTempRaw = 0;
		int32_t iSumCount = ENCODER_COUNT;
		if (g_axis.fbdk.uAngleRaw >= g_axis.posCtrl.uOffsetAngleRaw)
		{
			uTempRaw = g_axis.fbdk.uAngleRaw - g_axis.posCtrl.uOffsetAngleRaw;
		}
		else
		{
			uTempRaw = g_axis.fbdk.uAngleRaw - g_axis.posCtrl.uOffsetAngleRaw + iSumCount;
		}
		float fElecAngleUnwrapped = (float)uTempRaw / (float)iSumCount;
		float fMechAngle = fElecAngleUnwrapped * (float)POLE_PAIR_NUM;;
		*pAngle = FIXP30(fmodf(fMechAngle, 1.0f));
	}
}

/**
  * @brief  获取编码器角度原始值
  *
 */
uint16_t Get_Angle_Raw()
{
	return g_axis.fbdk.uAngleRaw;
}

/**
 * @brief  M法测速+低通滤波
 */
void Sensor_Update()
{

}

/**
 * @brief  卡尔曼滤波测速
 */
void Sensor_Update_Kalman()
{
	float fCurEncoderRaw = (float)g_axis.fbdk.uAngleRaw;

	float fDiffRaw = fCurEncoderRaw - s_prevEncoderRaw;
	float fSumCount = (float)ENCODER_COUNT;

	if (fDiffRaw > (fSumCount / 2))
	{
		fDiffRaw -= fSumCount;  // 修正为负值
	}
	else if (fDiffRaw < -(fSumCount / 2))
	{
		fDiffRaw += fSumCount;  // 修正为正值
	}


	float fSamplePeriod = 1.0f / 1000.0f; // 1khz的采样周期

	float fSpeedRpmRaw = 0.0f;

	if (fabs(fDiffRaw) <= 4.0)
	{
		fSpeedRpmRaw = 0;
	}
	else
	{
		// 转速 (RPM) = (角度变化 / 65536) / 时间(s) * 60
		fSpeedRpmRaw = (fDiffRaw / fSumCount) / fSamplePeriod * 60.0f;
	}

	g_axis.fbdk.fSpeedKalman = Kalman_Filter_Calc(&g_motorSpeedKalmanFilter, fSpeedRpmRaw);

	s_prevEncoderRaw = fCurEncoderRaw;
}

/**
 * @brief  PLL锁相环测速
 */
void Sensor_Update_PLL()
{

}





