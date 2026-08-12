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
#include "pidregdqx_current.h"
#include "Utils/Pid/pidreg_speed.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef enum
{
	MC_CIA402_SWITCH_ON_DISABLED = 0U,
	MC_CIA402_READY_TO_SWITCH_ON,
	MC_CIA402_SWITCHED_ON,
	MC_CIA402_OPERATION_ENABLED,
	MC_CIA402_QUICK_STOP_ACTIVE,
	MC_CIA402_FAULT
} McCia402State_t;

static McCia402State_t s_cia402_state = MC_CIA402_SWITCH_ON_DISABLED;
static uint16_t s_cia402_controlword = 0U;
static int8_t s_cia402_mode = CIA402_MODE_PROFILE_VELOCITY;
static int32_t s_cia402_target_velocity = 0;
static int16_t s_cia402_target_torque = 0;
static int32_t s_cia402_target_position = 0;
static bool s_cia402_csp_target_pending = false;
static bool s_cia402_csp_target_received = false;
static uint16_t s_cia402_sync_timeout_ms = 100U;
static uint16_t s_cia402_sync_elapsed_ms = 0U;
static bool s_cia402_sync_seen = false;
static bool s_cia402_sync_timeout_active = false;
static uint32_t s_cia402_profile_velocity = 0U;
static uint32_t s_cia402_profile_acceleration = 0U;
static uint32_t s_cia402_profile_deceleration = 0U;
static uint32_t s_cia402_following_error_window = 0U;
static uint16_t s_cia402_following_error_time = 0U;
static int32_t s_cia402_following_error_actual = 0;

/* CiA 402 state machine sits on top of the FOC motor state. */
static bool MC_Cia402_IsAxisBusy(void)
{
	return (g_axis.state == AXIS_STATE_OFFSET_CALIB) ||
		   (g_axis.state == AXIS_STATE_ENCODER_CALIB) ||
		   (g_axis.state == AXIS_STATE_PARAM_CALIB) ||
		   (g_axis.state == AXIS_STATE_CURRENT_AUTOTUNE) ||
		   (g_axis.state == AXIS_STATE_SPEED_AUTOTUNE);
}

static uint8_t MC_PolePairsOrDefault(void)
{
	if (g_axis.uPolePairs == 0U)
	{
		return (uint8_t)POLE_PAIR_NUM;
	}

	return g_axis.uPolePairs;
}

static int32_t MC_Cia402_GetActualPositionCounts(void)
{
	return (int32_t)(g_axis.posCtrl.iAbsRawPos - g_axis.posCtrl.iZeroAngle);
}

static bool MC_Cia402_UsesTrajectory(int8_t mode)
{
	return (mode == CIA402_MODE_PROFILE_POSITION);
}

static bool MC_Cia402_IsCspMode(void)
{
	return (s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_POSITION);
}

static bool MC_Cia402_CspReadyToStart(void)
{
	return (!MC_Cia402_IsCspMode()) || s_cia402_csp_target_received;
}

static void MC_Cia402_ResetSyncWatchdog(void)
{
	s_cia402_sync_elapsed_ms = 0U;
	s_cia402_sync_seen = false;
	s_cia402_sync_timeout_active = false;
}

static void MC_Cia402_HoldCurrentPosition(void)
{
	const float currentPosition = (float)MC_Cia402_GetActualPositionCounts();

	g_axis.posCtrl.fPosMeas = currentPosition;
	g_axis.posCtrl.fPosRef = currentPosition;
	g_axis.posCtrl.traj.inited = false;
}

static void MC_Cia402_LatchCspTarget(void)
{
	g_axis.posCtrl.fPosMeas = (float)MC_Cia402_GetActualPositionCounts();
	g_axis.posCtrl.fPosRef = (float)s_cia402_target_position;
	g_axis.posCtrl.traj.inited = false;
	s_cia402_csp_target_pending = false;
}

static void MC_Cia402_HandleSyncTimeout(void)
{
	MC_Cia402_HoldCurrentPosition();
	s_cia402_csp_target_pending = false;
	s_cia402_csp_target_received = false;
	s_cia402_sync_timeout_active = true;
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

	g_mc_calib_go_run_after_finish = 1U;
	g_bStartCurrentAutoTune = false;
	g_bStartSpeedAutoTune = false;
	MC_Set_Speed_Reference(0.0f);
	g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0f);
	g_axis.speedCtrl.iqOut_pu = FIXP30(0.0f);
	g_axis.currCtrl.refIdq.Q = FIXP30(0.0f);
	PIDREG_SPEED_setUi_pu(&g_axis.speedCtrl.PIDSpeed, FIXP30(0.0f));
	SpeedPos_ResetEstimator();

	if ((g_mc_calib_done_once != 0U) && g_axis.posCtrl.bCalibFlag)
	{
		g_axis.state = AXIS_STATE_RUN;
		return MC_SUCCESS;
	}

	g_axis.state = AXIS_STATE_OFFSET_CALIB;
	return MC_SUCCESS;
}

/**
  * @brief  电机停止运行
  */
MC_RetStatus_t MC_Stop_Motor(void)
{
	g_mc_calib_go_run_after_finish = 0U;
	g_bStartCurrentAutoTune = false;
	g_bStartSpeedAutoTune = false;
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
	if (g_axis.enCtrlMode == enControlMode)
	{
		return;
	}

	MC_Reset_Control_State();
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

void MC_Reset_Control_State(void)
{
	MC_Set_Speed_Reference(0.0f);
	g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
	g_axis.currCtrl.refIdq.Q = FIXP30(0.0f);
	g_axis.currCtrl.outIdq.D = FIXP30(0.0f);
	g_axis.currCtrl.outIdq.Q = FIXP30(0.0f);
	g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0f);
	g_axis.speedCtrl.iqOut_pu = FIXP30(0.0f);
	PIDREGDQX_CURRENT_setUiD_pu(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(0.0f));
	PIDREGDQX_CURRENT_setUiQ_pu(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(0.0f));
	PIDREG_SPEED_setUi_pu(&g_axis.speedCtrl.PIDSpeed, FIXP30(0.0f));
	SpeedPos_ResetEstimator();
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
	uint16_t command = controlword & 0x008FU;
	McCia402State_t previousState = s_cia402_state;
	McCia402State_t nextState = s_cia402_state;
	bool transitionValid = false;

	if (g_axis.state == AXIS_STATE_FAULT_NOW ||
		g_axis.state == AXIS_STATE_FAULT_OVER)
	{
		s_cia402_state = MC_CIA402_FAULT;
	}

	if (s_cia402_state == MC_CIA402_FAULT)
	{
		if ((controlword & 0x0080U) == 0U)
		{
			return MC_FAILED;
		}
		if (MC_Fault_Reset() != MC_SUCCESS)
		{
			return MC_FAILED;
		}
		s_cia402_state = MC_CIA402_SWITCH_ON_DISABLED;
		s_cia402_controlword = controlword;
		return MC_SUCCESS;
	}

	switch (s_cia402_state)
	{
		case MC_CIA402_SWITCH_ON_DISABLED:
			if (command == 0x0000U)
			{
				transitionValid = true;
			}
			else if (command == 0x0006U)
			{
				nextState = MC_CIA402_READY_TO_SWITCH_ON;
				transitionValid = true;
			}
			break;
		case MC_CIA402_READY_TO_SWITCH_ON:
			if (command == 0x0006U)
			{
				transitionValid = true;
			}
			else if (command == 0x0007U)
			{
				nextState = MC_CIA402_SWITCHED_ON;
				transitionValid = true;
			}
			else if (command == 0x0000U)
			{
				nextState = MC_CIA402_SWITCH_ON_DISABLED;
				transitionValid = true;
			}
			break;
		case MC_CIA402_SWITCHED_ON:
			if (command == 0x0007U)
			{
				transitionValid = true;
			}
			else if (command == 0x000FU)
			{
				nextState = MC_CIA402_OPERATION_ENABLED;
				transitionValid = true;
			}
			else if (command == 0x0006U)
			{
				nextState = MC_CIA402_READY_TO_SWITCH_ON;
				transitionValid = true;
			}
			else if (command == 0x0000U)
			{
				nextState = MC_CIA402_SWITCH_ON_DISABLED;
				transitionValid = true;
			}
			break;
		case MC_CIA402_OPERATION_ENABLED:
			if ((command & 0x0003U) == 0U)
			{
				nextState = MC_CIA402_SWITCH_ON_DISABLED;
				transitionValid = true;
			}
			else if ((controlword & 0x0004U) == 0U)
			{
				nextState = MC_CIA402_QUICK_STOP_ACTIVE;
				transitionValid = true;
			}
			else if (command == 0x000FU)
			{
				transitionValid = true;
			}
			else if (command == 0x0007U)
			{
				nextState = MC_CIA402_SWITCHED_ON;
				transitionValid = true;
			}
			else if (command == 0x0006U)
			{
				nextState = MC_CIA402_READY_TO_SWITCH_ON;
				transitionValid = true;
			}
			break;
		case MC_CIA402_QUICK_STOP_ACTIVE:
			if (command == 0x000FU)
			{
				nextState = MC_CIA402_OPERATION_ENABLED;
				transitionValid = true;
			}
			else if (command == 0x0007U)
			{
				nextState = MC_CIA402_SWITCHED_ON;
				transitionValid = true;
			}
			else if (command == 0x0006U)
			{
				nextState = MC_CIA402_READY_TO_SWITCH_ON;
				transitionValid = true;
			}
			else if (command == 0x0000U)
			{
				nextState = MC_CIA402_SWITCH_ON_DISABLED;
				transitionValid = true;
			}
			break;
		default:
			break;
	}

	if (!transitionValid)
	{
		return MC_FAILED;
	}

	if (nextState == MC_CIA402_OPERATION_ENABLED &&
		s_cia402_state != MC_CIA402_OPERATION_ENABLED)
	{
		bool motorAlreadyStarted = (g_axis.state == AXIS_STATE_RUN);

		/*
		 * A CiA 402 enable request is also valid while the application is
		 * completing its startup calibration.  Do not restart or abort that
		 * operation; the existing calibration flow will enter RUN when it
		 * finishes.
		 */
		if (MC_Cia402_IsCspMode())
		{
			MC_Cia402_HoldCurrentPosition();
		}

		if (!motorAlreadyStarted &&
			MC_Cia402_CspReadyToStart() &&
			!MC_Cia402_IsAxisBusy() &&
			(MC_Start_Motor() != MC_SUCCESS))
		{
			return MC_FAILED;
		}
	}
	else if (((previousState == MC_CIA402_OPERATION_ENABLED) &&
			  (nextState != MC_CIA402_OPERATION_ENABLED)) ||
			 ((nextState == MC_CIA402_QUICK_STOP_ACTIVE) &&
			  (previousState != MC_CIA402_QUICK_STOP_ACTIVE)) ||
			 ((nextState == MC_CIA402_SWITCH_ON_DISABLED) &&
			  (previousState != MC_CIA402_SWITCH_ON_DISABLED)))
	{
		/*
		 * 0x0006 and 0x0007 are intermediate state transitions.  Calling
		 * MC_Stop_Motor() for them would cancel a calibration that was
		 * started before the CiA 402 sequence was completed.
		 */
		(void)MC_Stop_Motor();
	}

	s_cia402_controlword = controlword;
	s_cia402_state = nextState;
	return MC_SUCCESS;
}

void MC_Cia402_ResetState(void)
{
	s_cia402_state = MC_CIA402_SWITCH_ON_DISABLED;
	s_cia402_controlword = 0U;
	s_cia402_mode = CIA402_MODE_PROFILE_VELOCITY;
	s_cia402_target_velocity = 0;
	s_cia402_target_torque = 0;
	s_cia402_target_position = 0;
	s_cia402_csp_target_pending = false;
	s_cia402_csp_target_received = false;
	MC_Cia402_ResetSyncWatchdog();
	s_cia402_profile_velocity = 0U;
	s_cia402_profile_acceleration = 0U;
	s_cia402_profile_deceleration = 0U;
	s_cia402_following_error_window = 0U;
	s_cia402_following_error_time = 0U;
	s_cia402_following_error_actual = 0;
	(void)MC_Stop_Motor();
	MC_Set_Control_Mode(CTRL_MODE_SPEED);
	g_axis.posCtrl.traj.bEnable = false;
	g_axis.posCtrl.traj.inited = false;
	g_axis.posCtrl.fPosRef = (float)MC_Cia402_GetActualPositionCounts();
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

	if (g_axis.state == AXIS_STATE_FAULT_NOW ||
		g_axis.state == AXIS_STATE_FAULT_OVER)
	{
		s_cia402_state = MC_CIA402_FAULT;
	}

	switch (s_cia402_state)
	{
		case MC_CIA402_SWITCH_ON_DISABLED:
			sw = 0x0040U;
			break;
		case MC_CIA402_READY_TO_SWITCH_ON:
			sw = 0x0021U;
			break;
		case MC_CIA402_SWITCHED_ON:
			sw = 0x0023U;
			break;
		case MC_CIA402_OPERATION_ENABLED:
			sw = 0x0027U;
			break;
		case MC_CIA402_QUICK_STOP_ACTIVE:
			sw = 0x0007U;
			break;
		case MC_CIA402_FAULT:
			sw = 0x0008U;
			break;
		default:
			break;
	}

	if (s_cia402_mode == CIA402_MODE_PROFILE_VELOCITY ||
		s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_VELOCITY)
	{
		int32_t actualVelocity;
		uint8_t polePairs = MC_PolePairsOrDefault();

		actualVelocity = (int32_t)(FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) *
			FREQUENCY_SCALE * 60.0f / (float)polePairs);
		if ((s_cia402_state == MC_CIA402_OPERATION_ENABLED) &&
			(labs(actualVelocity - s_cia402_target_velocity) <= 10L))
		{
			sw |= (1U << 10);
		}
	}
	else if (s_cia402_mode == CIA402_MODE_PROFILE_POSITION ||
			 s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_POSITION)
	{
		sw |= (1U << 12);
		{
			float posError = fabsf(g_axis.posCtrl.fPosRef -
				(float)(g_axis.posCtrl.iAbsRawPos - g_axis.posCtrl.iZeroAngle));
			float posWindow = (s_cia402_following_error_window > 0U) ?
				(float)s_cia402_following_error_window : 2.0f;

			if (posError <= posWindow)
			{
				sw |= (1U << 10);
			}
		}
	}
	else if (s_cia402_mode == CIA402_MODE_PROFILE_TORQUE ||
			 s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_TORQUE)
	{
		sw |= (1U << 8);
	}

	return sw;
}

void MC_Cia402_OnSync(uint8_t syncCounter)
{
	(void)syncCounter;

	s_cia402_sync_elapsed_ms = 0U;
	s_cia402_sync_seen = true;
	s_cia402_sync_timeout_active = false;

	if (s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_POSITION)
	{
		if (s_cia402_csp_target_pending)
		{
			MC_Cia402_LatchCspTarget();
		}
		else
		{
			g_axis.posCtrl.fPosMeas = (float)MC_Cia402_GetActualPositionCounts();
		}
	}
	else if (s_cia402_mode == CIA402_MODE_PROFILE_POSITION)
	{
		g_axis.posCtrl.fPosMeas = (float)MC_Cia402_GetActualPositionCounts();
	}
}

void MC_Cia402_Service1ms(bool canopenOperational)
{
	if (!MC_Cia402_IsCspMode() ||
		!canopenOperational ||
		(s_cia402_state != MC_CIA402_OPERATION_ENABLED))
	{
		MC_Cia402_ResetSyncWatchdog();
		return;
	}

	if (s_cia402_sync_elapsed_ms < 0xFFFFU)
	{
		s_cia402_sync_elapsed_ms++;
	}

	if (!s_cia402_sync_timeout_active &&
		(s_cia402_sync_seen || s_cia402_csp_target_pending || s_cia402_csp_target_received) &&
		(s_cia402_sync_elapsed_ms >= s_cia402_sync_timeout_ms))
	{
		MC_Cia402_HandleSyncTimeout();
	}
}

int8_t MC_Cia402_GetMode(void)
{
	return s_cia402_mode;
}

uint16_t MC_Cia402_GetControlword(void)
{
	return s_cia402_controlword;
}

bool MC_Cia402_ReadObject(uint16_t index, uint8_t subIndex,
						  uint8_t *value, uint8_t *size)
{
	int32_t actualVelocity;

	if ((value == NULL) || (size == NULL))
	{
		return false;
	}
	switch (index)
	{
		case 0x1001U:
			if (subIndex != 0U)
			{
				return false;
			}
			value[0] = (g_axis.error == AXIS_ERROR_NONE) ? 0U : 1U;
			*size = 1U;
			return true;
		case 0x6040U:
			if (subIndex != 0U)
			{
				return false;
			}
			value[0] = (uint8_t)s_cia402_controlword;
			value[1] = (uint8_t)(s_cia402_controlword >> 8);
			*size = 2U;
			return true;
		case 0x6041U:
		{
			if (subIndex != 0U)
			{
				return false;
			}
			uint16_t statusword = MC_Get_Cia402_Statusword();
			value[0] = (uint8_t)statusword;
			value[1] = (uint8_t)(statusword >> 8);
			*size = 2U;
			return true;
		}
		case 0x6060U:
		case 0x6061U:
			if (subIndex != 0U)
			{
				return false;
			}
			value[0] = (uint8_t)s_cia402_mode;
			*size = 1U;
			return true;
		case 0x60FFU:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_target_velocity, 4U);
			*size = 4U;
			return true;
		case 0x6072U:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_target_torque, 2U);
			*size = 2U;
			return true;
		case 0x6081U:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_profile_velocity, 4U);
			*size = 4U;
			return true;
		case 0x6083U:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_profile_acceleration, 4U);
			*size = 4U;
			return true;
		case 0x6084U:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_profile_deceleration, 4U);
			*size = 4U;
			return true;
		case 0x6065U:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_following_error_window, 4U);
			*size = 4U;
			return true;
		case 0x6066U:
			if (subIndex != 0U)
			{
				return false;
			}
			(void)memcpy(value, &s_cia402_following_error_time, 2U);
			*size = 2U;
			return true;
		case 0x60F4U:
			if (subIndex != 0U)
			{
				return false;
			}
			if ((s_cia402_mode == CIA402_MODE_PROFILE_POSITION) ||
				(s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_POSITION))
			{
				s_cia402_following_error_actual =
					(int32_t)g_axis.posCtrl.fPosRef -
					(int32_t)(g_axis.posCtrl.iAbsRawPos - g_axis.posCtrl.iZeroAngle);
			}
			else
			{
				uint8_t polePairs = MC_Get_Pole_Pairs();
				int32_t actualVelocityNow = (int32_t)(FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) *
					FREQUENCY_SCALE * 60.0f /
					(float)((polePairs == 0U) ? 1U : polePairs));
				s_cia402_following_error_actual = s_cia402_target_velocity - actualVelocityNow;
			}
			(void)memcpy(value, &s_cia402_following_error_actual, 4U);
			*size = 4U;
			return true;
		case 0x606CU:
			if (subIndex != 0U)
			{
				return false;
			}
			{
				uint8_t polePairs = MC_Get_Pole_Pairs();
				actualVelocity = (int32_t)(FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) *
					FREQUENCY_SCALE * 60.0f /
					(float)((polePairs == 0U) ? 1U : polePairs));
			}
			(void)memcpy(value, &actualVelocity, 4U);
			*size = 4U;
			return true;
		case 0x607AU:
		{
			if (subIndex != 0U)
			{
				return false;
			}
			int32_t targetPosition = s_cia402_target_position;
			(void)memcpy(value, &targetPosition, 4U);
			*size = 4U;
			return true;
		}
		case 0x6064U:
		{
			if (subIndex != 0U)
			{
				return false;
			}
			int32_t actualPosition = (int32_t)
				(g_axis.posCtrl.iAbsRawPos - g_axis.posCtrl.iZeroAngle);
			(void)memcpy(value, &actualPosition, 4U);
			*size = 4U;
			return true;
		}
		default:
			return MC_Cia402Ext_ReadObject(index, subIndex, value, size);
	}
}

bool MC_Cia402_WriteObject(uint16_t index, uint8_t subIndex,
						   const uint8_t *value, uint8_t size)
{
	int8_t mode;
	int32_t targetVelocity;
	int16_t targetTorque;

	if (value == NULL)
	{
		return false;
	}
	switch (index)
	{
		case 0x6040U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 2U)
			{
				return false;
			}
			return MC_Apply_Cia402_Controlword((uint16_t)value[0] |
				((uint16_t)value[1] << 8)) == MC_SUCCESS;
		case 0x6060U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 1U)
			{
				return false;
			}
			mode = (int8_t)value[0];
			if ((mode != CIA402_MODE_PROFILE_POSITION) &&
				(mode != CIA402_MODE_PROFILE_VELOCITY) &&
				(mode != CIA402_MODE_PROFILE_TORQUE) &&
				(mode != CIA402_MODE_CYCLIC_SYNC_POSITION) &&
				(mode != CIA402_MODE_CYCLIC_SYNC_VELOCITY) &&
				(mode != CIA402_MODE_CYCLIC_SYNC_TORQUE))
			{
				return false;
			}
			s_cia402_mode = mode;
			if ((mode == CIA402_MODE_PROFILE_POSITION) ||
				(mode == CIA402_MODE_CYCLIC_SYNC_POSITION))
			{
				MC_Set_Control_Mode(CTRL_MODE_POSITION);
				g_axis.posCtrl.traj.bEnable = MC_Cia402_UsesTrajectory(mode);
				g_axis.posCtrl.traj.inited = false;
				if (mode == CIA402_MODE_CYCLIC_SYNC_POSITION)
				{
					MC_Cia402_HoldCurrentPosition();
					s_cia402_csp_target_pending = false;
					s_cia402_csp_target_received = false;
					MC_Cia402_ResetSyncWatchdog();
				}
				else
				{
					g_axis.posCtrl.fPosRef = (float)s_cia402_target_position;
					s_cia402_csp_target_pending = false;
					s_cia402_csp_target_received = false;
					MC_Cia402_ResetSyncWatchdog();
				}
			}
			else if ((mode == CIA402_MODE_PROFILE_TORQUE) ||
					 (mode == CIA402_MODE_CYCLIC_SYNC_TORQUE))
			{
				MC_Set_Control_Mode(CTRL_MODE_TORQUE);
				g_axis.posCtrl.traj.bEnable = false;
				s_cia402_csp_target_pending = false;
				s_cia402_csp_target_received = false;
				MC_Cia402_ResetSyncWatchdog();
			}
			else
			{
				MC_Set_Control_Mode(CTRL_MODE_SPEED);
				g_axis.posCtrl.traj.bEnable = false;
				s_cia402_csp_target_pending = false;
				s_cia402_csp_target_received = false;
				MC_Cia402_ResetSyncWatchdog();
			}
			return true;
		case 0x60FFU:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 4U)
			{
				return false;
			}
			(void)memcpy(&targetVelocity, value, 4U);
			s_cia402_target_velocity = targetVelocity;
			MC_Set_Speed_Reference((float)targetVelocity);
			s_cia402_following_error_actual = 0;
			return true;
		case 0x6071U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 2U)
			{
				return false;
			}
			(void)memcpy(&targetTorque, value, 2U);
			s_cia402_target_torque = targetTorque;
			return MC_Set_Torque_Reference((float)targetTorque / 1000.0f) == MC_SUCCESS;
		case 0x6072U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 2U)
			{
				return false;
			}
			(void)memcpy(&targetTorque, value, 2U);
			s_cia402_target_torque = targetTorque;
			return true;
		case 0x6081U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 4U)
			{
				return false;
			}
			(void)memcpy(&s_cia402_profile_velocity, value, 4U);
			return true;
		case 0x6083U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 4U)
			{
				return false;
			}
			(void)memcpy(&s_cia402_profile_acceleration, value, 4U);
			MC_Set_Speed_Ramp((float)s_cia402_profile_acceleration);
			return true;
		case 0x6084U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 4U)
			{
				return false;
			}
			(void)memcpy(&s_cia402_profile_deceleration, value, 4U);
			return true;
		case 0x6065U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 4U)
			{
				return false;
			}
			(void)memcpy(&s_cia402_following_error_window, value, 4U);
			return true;
		case 0x6066U:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 2U)
			{
				return false;
			}
			(void)memcpy(&s_cia402_following_error_time, value, 2U);
			return true;
		case 0x607AU:
			if (subIndex != 0U)
			{
				return false;
			}
			if (size != 4U)
			{
				return false;
			}
			{
				int32_t targetPosition;

				(void)memcpy(&targetPosition, value, 4U);
				s_cia402_target_position = targetPosition;
				if (s_cia402_mode == CIA402_MODE_CYCLIC_SYNC_POSITION)
				{
					s_cia402_csp_target_received = true;
					s_cia402_csp_target_pending = true;
					s_cia402_sync_timeout_active = false;

					if ((s_cia402_state == MC_CIA402_OPERATION_ENABLED) &&
						(g_axis.state != AXIS_STATE_RUN) &&
						!MC_Cia402_IsAxisBusy())
					{
						if (MC_Start_Motor() != MC_SUCCESS)
						{
							return false;
						}
					}
				}
				else
				{
					g_axis.posCtrl.fPosRef = (float)targetPosition;
					g_axis.posCtrl.traj.inited = false;
					s_cia402_csp_target_pending = false;
				}
			}
			return true;
		default:
			return MC_Cia402Ext_WriteObject(index, subIndex, value, size);
	}
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
