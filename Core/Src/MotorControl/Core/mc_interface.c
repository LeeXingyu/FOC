/*
 * mc_interface.c
 *
 *  Created on: Apr 20, 2026
 *      Author: Administrator
 */

#include "mc_interface.h"
#include "main.h"
#include "motor_parameters.h"

static uint8_t MC_PolePairsOrDefault(void)
{
	if (g_axis.uPolePairs == 0U)
	{
		return (uint8_t)POLE_PAIR_NUM;
	}

	return g_axis.uPolePairs;
}

/**
  * @brief  电机开始运行
  */
MC_RetStatus_t MC_Start_Motor(void)
{
	if (g_axis.state != AXIS_STATE_IDLE)
	{
		return MC_FAILED;
	}

	/* 只有基础校准完成后，才允许从 IDLE 进入 RUN。 */
	if (g_axis.posCtrl.bCalibFlag == false)
	{
		return MC_FAILED;
	}

	g_axis.state = AXIS_STATE_RUN;
	return MC_SUCCESS;
}

/**
  * @brief  电机停止运行
  */
MC_RetStatus_t MC_Stop_Motor(void)
{
	g_axis.state = AXIS_STATE_IDLE;

	return MC_SUCCESS;
}

/**
  * @brief  设置控制模式
  */
void MC_Set_Control_Mode(ControlMode_En enControlMode)
{
	g_axis.enCtrlMode = enControlMode;
}

/**
  * @brief  电流校准
  */
void MC_Start_Curr_Offset_Cali()
{

}

/**
  * @brief  零点校准
  */
void MC_Zero_Calibration()
{

}

/**
  * @brief  设置id、iq幅值（开环使用）
  * @param  dutyCycle 归一化幅值
  */
void MC_Set_Duty_Cycle(Duty_Ddq_t dutyCycle)
{
	g_axis.currCtrl.refIdq.D = dutyCycle.D;
	g_axis.currCtrl.refIdq.Q = dutyCycle.Q;
}

void MC_Set_Speed_Ramp(float fRamp)
{
	float fHzPerIsr = fRamp / SPEED_CONTROL_RATE;
	g_axis.speedCtrl.speedRamp_pu = FIXP30(fHzPerIsr / FREQUENCY_SCALE);
}

void MC_Set_Speed_Kp(float fKp)
{
	PIDREG_SPEED_setKp_si(&g_axis.speedCtrl.PIDSpeed, fKp);
}

void MC_Set_Speed_Ki(float fKi)
{
	PIDREG_SPEED_setKi_si(&g_axis.speedCtrl.PIDSpeed, fKi);
}

void MC_Set_Speed_Reference(float fRefSpeed)
{
	float fFreqHz = (fRefSpeed * (float)MC_PolePairsOrDefault()) / 60.0f;
	g_axis.speedCtrl.speedRef_pu = FIXP30( fFreqHz / FREQUENCY_SCALE);
}

uint8_t MC_Get_Pole_Pairs(void)
{
	return MC_PolePairsOrDefault();
}

MC_RetStatus_t MC_Set_Pole_Pairs(uint8_t polePairs)
{
	if (polePairs == 0U)
	{
		return MC_FAILED;
	}

	g_axis.uPolePairs = polePairs;
	return MC_SUCCESS;
}
