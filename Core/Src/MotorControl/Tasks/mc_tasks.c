/*
 * mc_tasks.c
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#include "mc_tasks.h"
#include "fixpmath.h"
#include "main.h"
#include "curr_fbdk.h"
#include "speed_pos_fbdk.h"
#include "foc.h"
#include "motor_parameters.h"
#include "encoder.h"
#include "param_identify.h"

static uint16_t FOC_Controller(void);
static void Param_Calib_Handle(void);
static void Param_Calib_SampleFeedback(void);
static void Param_Calib_ApplyDuty(void);
/**
 * @brief  高频任务（tim1定时器触发16khz+）
 */
void High_Frequency_Task(void)
{
    // 异步读取两个编码器（SPI1 + SPI3）
    (void)Start_Encoder_Read(ENC_ID_MOTOR);
    //(void)Start_Encoder_Read(ENC_ID_LOAD);
    Get_Vbus_Measurements(g_axis.pPWMCHandle, &g_axis.busVoltage);
    uint16_t uFOCreturn;

    // 已经初始化完毕
    if (g_axis.bMCBootCompleted)
    {
        switch (g_axis.state)
        {
            case AXIS_STATE_IDLE:
                Idle_Handle();
                break;

            case AXIS_STATE_OFFSET_CALIB:
                Offset_Calib_Handle();
                break;

            case AXIS_STATE_ENCODER_CALIB:
                Offset_Encoder_Handle();
                break;

            case AXIS_STATE_PARAM_CALIB:
                Param_Calib_Handle();
                break;

            case AXIS_STATE_RUN:
                Run_Handle();
                break;

            case AXIS_STATE_FAULT_NOW:
            case AXIS_STATE_FAULT_OVER:
                SwitchOff_PWM(g_axis.pPWMCHandle);
				//DRV8353_UpdateFaultStatus();
                break;

            default:
                break;
        }
    }
}

/**
 * @brief  中频任务（定时器触发，1khz）
 */
void Medium_Frequency_Task()
{
	

	if (g_axis.bMCBootCompleted)
	{
		// 圈数计算
		Circle_Update();

		// 测速
		Calc_Speed(&g_axis.speedCtrl.speedMeas_pu);
	}
	
	//ParamId_ModuleBackgroundService();
}

/**
 * @brief  状态机转换
 */
void Axis_State_Machine()
{

}

void MC_Calib_Init(void)
{
	ParamId_ModuleInit();
}

MC_RetStatus_t MC_Calib_StartChain(void)
{
	if (g_axis.state != AXIS_STATE_IDLE)
	{
		return MC_FAILED;
	}

	g_mc_calib_go_run_after_finish = 0U;
	g_axis.state = AXIS_STATE_OFFSET_CALIB;
	return MC_SUCCESS;
}

MC_RetStatus_t MC_Calib_StartParam(ParamIdStep_t step)
{
	ParamIdRet_t ret;

	if (step != PARAM_ID_STEP_ALL)
	{
		return MC_FAILED;
	}

	if (g_axis.state != AXIS_STATE_IDLE)
	{
		return MC_FAILED;
	}

	if (g_axis.posCtrl.bCalibFlag == false)
	{
		return MC_FAILED;
	}

	ret = ParamId_ModuleStart(step);
	if (ret != PARAM_ID_OK)
	{
		return MC_FAILED;
	}

	g_axis.state = AXIS_STATE_PARAM_CALIB;
	return MC_SUCCESS;
}

MC_RetStatus_t MC_Calib_StopParam(void)
{
	ParamIdRet_t ret = ParamId_ModuleStop();

	if ((ret != PARAM_ID_OK) && (ret != PARAM_ID_ERR_NOT_RUNNING))
	{
		return MC_FAILED;
	}

	if (g_axis.state == AXIS_STATE_PARAM_CALIB)
	{
		g_axis.state = AXIS_STATE_IDLE;
	}

	return MC_SUCCESS;
}

ParamIdState_t MC_Calib_GetParamState(void)
{
	return ParamId_ModuleGetState();
}

const ParamIdResult_t *MC_Calib_GetParamResult(void)
{
	return ParamId_ModuleGetResult();
}

/**
 * @brief  空闲状态处理
 */
void Idle_Handle()
{
	// 关闭pwm输出，不采电流，不跑控制链
	SwitchOff_PWM(g_axis.pPWMCHandle);

	g_axis.posCtrl.uCalibCount = 0;

	PIDREGDQX_CURRENT_setUiD_pu(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(0.0f));
	PIDREGDQX_CURRENT_setUiQ_pu(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(0.0f));
	PIDREG_SPEED_setUi_pu(&g_axis.speedCtrl.PIDSpeed, FIXP30(0.0f));

//	Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas);
}

/**
 * @brief  电流偏移校准
 */
void Offset_Calib_Handle()
{
	if (g_axis.pPWMCHandle->bIsMeasuringOffset == false)
	{
		g_axis.pPWMCHandle->bIsMeasuringOffset = true;
		g_axis.pPWMCHandle->uCalibCount = 0;
		g_axis.pPWMCHandle->fCalibRsum = 0.0f;
		g_axis.pPWMCHandle->fCalibSsum = 0.0f;
		g_axis.pPWMCHandle->fCalibTsum = 0.0f;
	}
	else
	{
		Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas);

		g_axis.pPWMCHandle->uCalibCount++;
		g_axis.pPWMCHandle->fCalibRsum += FIXP30_toF(g_axis.currCtrl.IrstMeas.R) * CURRENT_SCALE;
		g_axis.pPWMCHandle->fCalibSsum += FIXP30_toF(g_axis.currCtrl.IrstMeas.S) * CURRENT_SCALE;
		g_axis.pPWMCHandle->fCalibTsum += FIXP30_toF(g_axis.currCtrl.IrstMeas.T) * CURRENT_SCALE;

		if (g_axis.pPWMCHandle->uCalibCount >= CURR_CALIB_COUNT)
		{
			float fOffsetIr = g_axis.pPWMCHandle->fCalibRsum / (float)g_axis.pPWMCHandle->uCalibCount;
			float fOffsetIs = g_axis.pPWMCHandle->fCalibSsum / (float)g_axis.pPWMCHandle->uCalibCount;
			float fOffsetIt = g_axis.pPWMCHandle->fCalibTsum / (float)g_axis.pPWMCHandle->uCalibCount;

			g_axis.pPWMCHandle->offsetIr = FIXP30(fOffsetIr / CURRENT_SCALE);
			g_axis.pPWMCHandle->offsetIs = FIXP30(fOffsetIs / CURRENT_SCALE);
			g_axis.pPWMCHandle->offsetIt = FIXP30(fOffsetIt / CURRENT_SCALE);

			g_axis.state = AXIS_STATE_ENCODER_CALIB;
			g_axis.pPWMCHandle->bIsMeasuringOffset = false;
			//printf("Offset_Calib_Handle Over\n");
		}
	}

	Write_TIM_Registers(g_axis.pPWMCHandle);
}

/**
 * @brief  编码器校准
 */
void Offset_Encoder_Handle()
{
	Duty_Dab_t dutyAb;
	Duty_Drst_t dutyRst;

	Currents_Idq_t refIdq;
	refIdq.D = FIXP30(0.05f);
	refIdq.Q = FIXP30(0.00f);

	// 电气角度
	fixp30_t anglePark_pu;
	Get_Angle(&anglePark_pu);

	// 获取测量角度的余弦、正弦值
	FIXP_CosSin_t cossinPwm;
	FIXP30_CosSinPU(anglePark_pu, &cossinPwm);

	// 反park变换
	Inv_Park_Duty(refIdq, &cossinPwm, &dutyAb);

	// 反Clarkr变换
	Inv_Clarke_Duty(dutyAb, &dutyRst);

	// 打开PWM输出
	SwitchOn_PWM(g_axis.pPWMCHandle);

	// 空间矢量调制
	Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);

	g_axis.posCtrl.uCalibCount++;
	if (g_axis.posCtrl.uCalibCount >= ENCODER_CALIB_COUNT)
	{
		PID_Reset(&g_axis.currCtrl.pidId, 0.0f);
		PID_Reset(&g_axis.currCtrl.pidIq, 0.0f);
		g_axis.posCtrl.uOffsetAngleRawNative = Get_Angle_RawNative();
		g_axis.posCtrl.uCalibCount = 0;
		g_axis.posCtrl.bCalibFlag = true;
		MC_Set_Speed_Reference(0.0f);
		MC_Set_Control_Mode(CTRL_MODE_SPEED);
		g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
		g_axis.currCtrl.refIdq.Q = FIXP30(0.0f);
		g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0f);
		g_axis.speedCtrl.iqOut_pu = FIXP30(0.0f);
		PIDREG_SPEED_setUi_pu(&g_axis.speedCtrl.PIDSpeed, FIXP30(0.0f));
		g_mc_calib_done_once = 1U;
		if (g_mc_calib_go_run_after_finish != 0U)
		{
			g_mc_calib_go_run_after_finish = 0U;
			g_axis.state = AXIS_STATE_RUN;
		}
		else
		{
			g_axis.state = AXIS_STATE_IDLE;
		}
	}
}

static void Param_Calib_Handle(void)
{
	ParamIdState_t state;

	Param_Calib_SampleFeedback();
	Param_Calib_ApplyDuty();
	ParamId_ModuleService();
	state = ParamId_ModuleGetState();

	if (state == PARAM_ID_STATE_DONE)
	{
		Duty_Ddq_t duty = {0};
		MC_Set_Speed_Reference(0.0f);
		MC_Set_Duty_Cycle(duty);
		MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
		g_axis.state = AXIS_STATE_IDLE;
	}
	else if (state == PARAM_ID_STATE_FAULT)
	{
		g_axis.error = (AxisError_t)(g_axis.error | AXIS_ERROR_CALIBRATION_FAILED);
		g_axis.state = AXIS_STATE_FAULT_NOW;
	}
}

static void Param_Calib_SampleFeedback(void)
{
	fixp30_t anglePark_pu;
	FIXP_CosSin_t cossinPark;

	Get_Vbus_Measurements(g_axis.pPWMCHandle, &g_axis.busVoltage);
	Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas);

	Get_Angle(&anglePark_pu);
	FIXP30_CosSinPU(anglePark_pu, &cossinPark);
	Clarke_Current(g_axis.currCtrl.IrstMeas, &g_axis.currCtrl.calcIab);
	Park_Current(g_axis.currCtrl.calcIab, &cossinPark, &g_axis.currCtrl.calcIdq);
}

static void Param_Calib_ApplyDuty(void)
{
	fixp30_t anglePark_pu;
	FIXP_CosSin_t cossinPark;
	Duty_Dab_t dutyAB;
	Duty_Drst_t dutyRst;

	Get_Angle(&anglePark_pu);
	FIXP30_CosSinPU(anglePark_pu, &cossinPark);
	Inv_Park_Duty(g_axis.currCtrl.refIdq, &cossinPark, &dutyAB);
	Inv_Clarke_Duty(dutyAB, &dutyRst);
	SwitchOn_PWM(g_axis.pPWMCHandle);
	Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);
}

void Run_Handle()
{
	SwitchOn_PWM(g_axis.pPWMCHandle);

	switch(g_axis.enCtrlMode)
	{
		case CTRL_MODE_OPEN_LOOP:
			// 开环
			Open_Loop_Control();
			break;
		default:
			// 闭环：电流环、速度环、位置环控制
			FOC_Control();
			break;
	}


}

/**
 * @brief 执行FOC的核心，即用于直流电流调节的控制器。参考坐标系变换根据主动速度传感器进行
 * @retval 如果FOC在下一个PWM更新事件之前结束，则返回MC_NO_FAULTS,否则返可MC_DURATION。
 */
inline uint16_t FOC_Controller(void)
{
	// 获取母线电压
	Get_Vbus_Measurements(g_axis.pPWMCHandle, &g_axis.busVoltage);

	// 获取转速
	Get_Speed(&g_axis.speedCtrl.speedMeas_pu);

	// 速度环(1khz)
	static int iCount = 0;
	iCount++;
	if (iCount == 16)
	{
		iCount = 0;
		Speed_Loop();
	}

	// 开环自增、减角度处理

	// 获取三相电流
	Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas);

	// 电流环

	// 空间矢量调制
	SetPhaseDuty(g_axis.pPWMCHandle);
	return;
}
