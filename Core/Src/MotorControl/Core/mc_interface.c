/*
 * mc_interface.c
 *
 *  Created on: Apr 20, 2026
 *      Author: Administrator
 */

#include "mc_interface.h"
#include "main.h"
#include "motor_parameters.h"
#include "MotorControl/Fbdk/speed_pos_fbdk.h"
#include "Utils/Pid/pidreg_speed.h"
#include <math.h>

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

	MC_Set_Speed_Reference(0.0f);
	g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0f);
	g_axis.speedCtrl.iqOut_pu = FIXP30(0.0f);
	g_axis.currCtrl.refIdq.Q = FIXP30(0.0f);
	PIDREG_SPEED_setUi_pu(&g_axis.speedCtrl.PIDSpeed, FIXP30(0.0f));
	SpeedPos_ResetEstimator();

	if (g_mc_calib_done_once != 0U)
	{
		g_mc_calib_go_run_after_finish = 0U;
		g_axis.state = AXIS_STATE_RUN;
	}
	else
	{
		g_mc_calib_go_run_after_finish = 1U;
		g_axis.state = AXIS_STATE_OFFSET_CALIB;
	}
	return MC_SUCCESS;
}

/**
  * @brief  电机停止运行
  */
MC_RetStatus_t MC_Stop_Motor(void)
{
	g_mc_calib_go_run_after_finish = 0U;
	MC_Set_Speed_Reference(0.0f);
	g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0f);
	g_axis.speedCtrl.iqOut_pu = FIXP30(0.0f);
	g_axis.currCtrl.refIdq.Q = FIXP30(0.0f);
	PIDREG_SPEED_setUi_pu(&g_axis.speedCtrl.PIDSpeed, FIXP30(0.0f));
	SpeedPos_ResetEstimator();
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

MC_RetStatus_t MC_Set_Torque_Reference(float fIqA)
{
	if (!isfinite(fIqA))
	{
		return MC_FAILED;
	}

	g_axis.currCtrl.refIdq.Q = FIXP30(fIqA / CURRENT_SCALE);
	return MC_SUCCESS;
}

MC_RetStatus_t MC_Apply_Cia402_Controlword(uint16_t controlword)
{
	/*
	 * Minimal CiA 402 behavior:
	 * - bit0/1/2/3: enable sequence
	 * - bit7: fault reset
	 * We only map the sequence to existing start/stop logic.
	 */
	if ((controlword & (1U << 7)) != 0U)
	{
		return MC_Fault_Reset();
	}

	if ((controlword & 0x000F) == 0x000F)
	{
		return MC_Start_Motor();
	}

	if ((controlword & 0x0006) == 0x0000)
	{
		return MC_Stop_Motor();
	}

	return MC_SUCCESS;
}

MC_RetStatus_t MC_Fault_Reset(void)
{
	if (g_axis.state == AXIS_STATE_FAULT_NOW)
	{
		g_axis.state = AXIS_STATE_FAULT_OVER;
	}

	if (g_axis.state == AXIS_STATE_FAULT_OVER)
	{
		g_axis.error = AXIS_ERROR_NONE;
		MC_Stop_Motor();
		g_axis.state = AXIS_STATE_IDLE;
		return MC_SUCCESS;
	}

	return MC_FAILED;
}

uint16_t MC_Get_Cia402_Statusword(void)
{
	uint16_t sw = 0U;

	switch (g_axis.state)
	{
		case AXIS_STATE_IDLE:
			sw |= (1U << 0);
			break;
		case AXIS_STATE_RUN:
			sw |= (1U << 0);
			sw |= (1U << 1);
			sw |= (1U << 2);
			sw |= (1U << 4);
			break;
		case AXIS_STATE_FAULT_NOW:
		case AXIS_STATE_FAULT_OVER:
			sw |= (1U << 3);
			break;
		default:
			break;
	}

	if (g_axis.enCtrlMode == CTRL_MODE_SPEED)
	{
		sw |= (1U << 10);
	}
	else if (g_axis.enCtrlMode == CTRL_MODE_POSITION)
	{
		sw |= (1U << 11);
	}
	else if (g_axis.enCtrlMode == CTRL_MODE_TORQUE)
	{
		sw |= (1U << 8);
	}

	return sw;
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
