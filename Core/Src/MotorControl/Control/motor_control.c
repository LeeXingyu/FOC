/*
 * motor_control.c
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#include "motor_control.h"
#include "main.h"
#include "mc_math.h"
#include "curr_fbdk.h"
#include "motor_parameters.h"
#include "fixpmath.h"
#include "mc_interface.h"
#include "encoder.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

CurrentFilter_t iqFilter;

//空载
#define KF_R 7.0f
#define KF_Q 0.01f
//带载
//#define KF_R 7.0f
//#define KF_Q 0.04f

void PID_All_Init()
{
	/* 初始化电流环PI */

	// mcsdk pid

	float fDutyLimit = BOARD_MAX_MODULATION;

	FIXPSCALED_floatToFIXPscaled(1000.0f / TF_REGULATION_RATE, &g_axis.currCtrl.busVoltageFilter); // 1000 (radians/s) filter for bus voltage compensation
	g_axis.currCtrl.busVoltageComp = FIXP24(1.0f);
	g_axis.currCtrl.busVoltageCompMax = FIXP24(20.0f);
	g_axis.currCtrl.busVoltageCompMin = FIXP24(1.0f);

	// mcsdk的电流环pid
	PIDREGDQX_CURRENT_init(&g_axis.currCtrl.pid_IdIqX_obj,
			CURRENT_SCALE,
			VOLTAGE_SCALE,
			TF_REGULATION_RATE,
			FREQUENCY_SCALE,
			fDutyLimit);

	PIDREGDQX_CURRENT_setKp_si(&g_axis.currCtrl.pid_IdIqX_obj, 0.15f);
	PIDREGDQX_CURRENT_setWi_si(&g_axis.currCtrl.pid_IdIqX_obj, 20.0f);
	PIDREGDQX_CURRENT_setOutputLimitsD(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(fDutyLimit * 0.95f), FIXP30(-fDutyLimit * 0.95f));
	PIDREGDQX_CURRENT_setOutputLimitsQ(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(fDutyLimit), FIXP30(-fDutyLimit));

	// mcsdk的速度环pid
	fixp30_t currentLimit_pu = FIXP30((float)PID_MAX_CURRENT / (float)CURRENT_SCALE);
	PIDREG_SPEED_init(&g_axis.speedCtrl.PIDSpeed, CURRENT_SCALE, FREQUENCY_SCALE, SPEED_CONTROL_RATE);
	PIDREG_SPEED_setOutputLimits(&g_axis.speedCtrl.PIDSpeed, currentLimit_pu, -currentLimit_pu);

	PIDREG_SPEED_setKp_si(&g_axis.speedCtrl.PIDSpeed, 1.0f);
	PIDREG_SPEED_setKi_si(&g_axis.speedCtrl.PIDSpeed, 4.0f);
}

/**
  * @brief  电机控制初始化
  */
void Motor_Control_Init(void)
{
	g_axis.state = AXIS_STATE_UNDEFINED;
	g_axis.bMCBootCompleted = false;

	g_axis.pPWMCHandle = &pwmcHandle;

	// 电流、电压采样初始化（定时器、采样配置）
	Sampling_Init();
	Init_Encoder();

	// 定时器开启
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);

	// 卡尔曼滤波初始化
	Kalman_Filter_Init(&g_motorSpeedKalmanFilter, KF_R, KF_Q);


	/* 开环参数初始化 */
	g_axis.posCtrl.hzToStepAngle_pu = FIXP30(FREQUENCY_SCALE / PWM_FREQUENCY);
	g_axis.posCtrl.openLoopAngle_pu = FIXP30(0.0f);
	// 默认幅值
	g_axis.currCtrl.refIdq.D = FIXP30(0.00f);
	g_axis.currCtrl.refIdq.Q = FIXP30(0.1f);
	// 默认控制状态
	g_axis.enCtrlMode = CTRL_MODE_OPEN_LOOP;

	g_axis.pPWMCHandle->uCalibCount = 0;

	// 位置相关
	g_axis.posCtrl.uOffsetAngleRaw = 0;
	g_axis.posCtrl.uOffsetAngleRawNative = 0U;
	g_axis.posCtrl.uCalibCount = 0;
	g_axis.posCtrl.bCalibFlag = false;

	// 速度相关
	float fElectricalFreqHz = ((float)POLE_PAIR_NUM * 50.0f) / 60.0f;
	g_axis.speedCtrl.speedRef_pu = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
	g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0F);

	g_axis.speedCtrl.speedRef_pu = FIXP30(fElectricalFreqHz / FREQUENCY_SCALE);
	MC_Set_Speed_Ramp(32.0f);

	// 初始化状态
	g_axis.state = AXIS_STATE_IDLE;
	g_axis.bMCBootCompleted = true;

	// 初始化pid
	PID_All_Init();

    iqFilter.fAlpha = 0.5f;
}

/**
  * @brief  foc控制
  */
uint16_t FOC_Control(void)
{

	// 获取三相电流
	Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas);

	// 获取母线电压
	Get_Vbus_Measurements(g_axis.pPWMCHandle, &g_axis.busVoltage);

	// 速度环输出目标电流
	static int iSpeedCount = 0;
	iSpeedCount++;
	if (iSpeedCount == 16)
	{
		Speed_Control(&g_axis.speedCtrl);
		g_axis.currCtrl.refIdq.Q = g_axis.speedCtrl.iqOut_pu;
		iSpeedCount = 0;
	}

	// 电流控制
	CurrCtrlInput_t currCtrlInput;
	currCtrlInput.Irst_in_pu.R = g_axis.currCtrl.IrstMeas.R;
	currCtrlInput.Irst_in_pu.S = g_axis.currCtrl.IrstMeas.S;
	currCtrlInput.Irst_in_pu.T = g_axis.currCtrl.IrstMeas.T;
	currCtrlInput.Udcbus_in_pu = g_axis.busVoltage;

	Curr_Control(&g_axis.currCtrl, &currCtrlInput);
  
	return 0;
}

/**
  * @brief  电流控制
  */
void Curr_Control(CurrCtrl_t *pCurrCtrl, CurrCtrlInput_t* pCurrCtrlInput)
{
	fixp24_t busVoltageComp = Bus_Voltage_Compensation(pCurrCtrl, pCurrCtrlInput->Udcbus_in_pu);
	PIDREGDQX_CURRENT_setCompensation(&g_axis.currCtrl.pid_IdIqX_obj, busVoltageComp);

	// 占空比
	Duty_Dab_t dutyAB;
	Duty_Drst_t dutyRst;

	// 电气角度
	fixp30_t anglePark_pu;
	Get_Angle(&anglePark_pu);

	// 获取测量角度的余弦、正弦值
	FIXP_CosSin_t cossinPark;
	FIXP30_CosSinPU(anglePark_pu, &cossinPark);
	// Clarke变换
	Clarke_Current(pCurrCtrlInput->Irst_in_pu, &pCurrCtrl->calcIab);

	// Park变换
	Park_Current(pCurrCtrl->calcIab, &cossinPark, &pCurrCtrl->calcIdq);

	CurrentFilter_Update(&iqFilter, FIXP30_toF(pCurrCtrl->calcIdq.Q) * CURRENT_SCALE);
	pCurrCtrl->calcIdq.Q = FIXP30(iqFilter.fCurr / (float)CURRENT_SCALE);

//	g_axis.currCtrl.refIdq.Q = FIXP30(0.08f / CURRENT_SCALE);

//	g_axis.currCtrl.outIdq.Q = PID_Run_FIXP30(
//	        &g_axis.currCtrl.pidIq,
//	        g_axis.currCtrl.refIdq.Q,
//			pCurrCtrl->calcIdq.Q,
//	        FIXP30(0.0f)
//	    );

	PIDREGDQX_CURRENT_setUiD_pu(&g_axis.currCtrl.pid_IdIqX_obj, FIXP30(0.0f));
	PIDREGDQX_CURRENT_run(
			&g_axis.currCtrl.pid_IdIqX_obj,
			FIXP30(0.0f),
			(g_axis.currCtrl.refIdq.Q - pCurrCtrl->calcIdq.Q),
			FIXP30(0.0f)
			);

	g_axis.currCtrl.outIdq.D = FIXP30(0.0f);
	g_axis.currCtrl.outIdq.Q = PIDREGDQX_CURRENT_getOutQ(&g_axis.currCtrl.pid_IdIqX_obj);

	// 角度补偿?
	Get_Angle(&anglePark_pu);

	// 获取测量角度的余弦、正弦值
	FIXP30_CosSinPU(anglePark_pu, &cossinPark);
	// 反Clarkr变换
	Inv_Park_Duty(g_axis.currCtrl.outIdq, &cossinPark, &dutyAB);

	// 反Clarkr变换
	Inv_Clarke_Duty(dutyAB, &dutyRst);

	// 空间矢量调制
	Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);
}

/**
  * @brief  速度控制
  */
void Speed_Control(SpeedCtrl_t *pSpeedCtrl)
{
	// 速度参考斜坡
	fixp30_t speedRamp = pSpeedCtrl->speedRamp_pu;
	fixp30_t speedDelta_pu = pSpeedCtrl->speedRef_pu - pSpeedCtrl->speedRefRamp_pu;
	pSpeedCtrl->speedRefRamp_pu += FIXP_sat(speedDelta_pu, speedRamp, -speedRamp);

//	pSpeedCtrl->speedRefRamp_pu = pSpeedCtrl->speedRef_pu;

	/* 位置环t型规划 */

	//
//	pSpeedCtrl->iqOut_pu = PID_Run_FIXP30(
//			&g_axis.speedCtrl.pid,
//			pSpeedCtrl->speedRefRamp_pu,
//			pSpeedCtrl->speedMeas_pu,
//			FIXP30(0.0f)
//		);

	fixp30_t speedError = pSpeedCtrl->speedRefRamp_pu - pSpeedCtrl->speedMeas_pu;
	fixp30_t IqRef_pu = PIDREG_SPEED_run(&g_axis.speedCtrl.PIDSpeed, speedError);

	fixp30_t IqRefCircleMax = FIXP30(12.0 / CURRENT_SCALE);

	pSpeedCtrl->iqOut_pu = FIXP_sat(IqRef_pu, IqRefCircleMax, -IqRefCircleMax);
}

fixp30_t Bus_Voltage_Compensation(CurrCtrl_t *pCurrCtrl, const fixp30_t udc_pu)
{
	fixp24_t busVoltageComp = pCurrCtrl->busVoltageComp;

	fixp24_t temp = FIXP(1.0f) - FIXP30_mpy(udc_pu, busVoltageComp);
	fixp24_t factor = FIXP_mpyFIXPscaled(busVoltageComp, &pCurrCtrl->busVoltageFilter);
	busVoltageComp += FIXP_mpy(temp,factor);
	busVoltageComp = FIXP_sat(busVoltageComp, pCurrCtrl->busVoltageCompMax, pCurrCtrl->busVoltageCompMin);
	pCurrCtrl->busVoltageComp = busVoltageComp;

	return busVoltageComp;
}

/**
 * @brief  开环旋转测试, V/F 控制（电压随频率同步变化）
 */
void Open_Loop_Control()
{
	// 打开PWM输出
	SwitchOn_PWM(g_axis.pPWMCHandle);

	// 电气角度
	fixp30_t anglePark_pu;

	// 获取三相电流
	Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas);

	FIXP_CosSin_t cossinPark;
	FIXP30_CosSinPU(anglePark_pu, &cossinPark);
	// Clarke变换
	Clarke_Current(g_axis.currCtrl.IrstMeas, &g_axis.currCtrl.calcIab);

	// Park变换
	Park_Current(g_axis.currCtrl.calcIab, &cossinPark, &g_axis.currCtrl.calcIdq);

	// 占空比
	Duty_Dab_t dutyAb;
	Duty_Drst_t dutyRst;

	// 自动更新开环角度
	Update_Open_Loop_Angle(&g_axis.posCtrl, g_axis.speedCtrl.speedRef_pu);

	// 获取电气角度
//	anglePark_pu = g_axis.posCtrl.openLoopAngle_pu;
	Get_Angle(&anglePark_pu);

	// 获取测量角度的余弦、正弦值
	FIXP_CosSin_t cossinPwm;
	FIXP30_CosSinPU(anglePark_pu, &cossinPwm);

	g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
	g_axis.currCtrl.refIdq.Q = FIXP30(0.1f);

	// 反park变换
	Inv_Park_Duty(g_axis.currCtrl.refIdq, &cossinPwm, &dutyAb);

	// 反Clarkr变换
	Inv_Clarke_Duty(dutyAb, &dutyRst);

	// 空间矢量调制
	Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);
}

/**
 * @brief  自动更新开环角度
 */
void Update_Open_Loop_Angle(PosCtrl_t *pHandle, fixp30_t speedRef)
{
	fixp30_t angleStep_pu = FIXP30_mpy(speedRef, pHandle->hzToStepAngle_pu);
	fixp30_t angle_pu = pHandle->openLoopAngle_pu;
	angle_pu += angleStep_pu;
	angle_pu &= (FIXP30(1.0f) - 1);
	pHandle->openLoopAngle_pu = angle_pu;
}
