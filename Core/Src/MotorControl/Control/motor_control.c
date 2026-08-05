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
#include "foc.h"
#include "motor_parameters.h"
#include "fixpmath.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "pid_position.h"
#include "param_identify.h"
#include "encoder.h"
#include "speed_pos_fbdk.h"
#include <math.h>

static void Update_Open_Loop_Angle(PosCtrl_t *pHandle, fixp30_t speedRef);
static void Curr_Pid_Init(void);
static void Speed_Pid_Init(void);
static void Position_Pid_Init(void);

static float s_iqRawDisplayA = 0.0f;
static float s_iqFilteredDisplayA = 0.0f;
CurrentFilter_t iqFilter;
volatile int32_t g_dbg_q_vector_sign = 0;
volatile uint8_t g_dbg_dir_consistent = 0U;

#define KF_R 7.0f
#define KF_Q 0.01f

static uint16_t s_speedCount = 0U;

void PID_All_Init(void)
{
    Curr_Pid_Init();
    Speed_Pid_Init();
    Position_Pid_Init();
}

static void Curr_Pid_Init(void)
{
    float fDutyLimit = BOARD_MAX_MODULATION;

    FIXPSCALED_floatToFIXPscaled(1000.0f / TF_REGULATION_RATE, &g_axis.currCtrl.busVoltageFilter);
    g_axis.currCtrl.busVoltageComp = FIXP24(1.0f);
    g_axis.currCtrl.busVoltageCompMax = FIXP24(20.0f);
    g_axis.currCtrl.busVoltageCompMin = FIXP24(1.0f);

    PIDREGDQX_CURRENT_init(&g_axis.currCtrl.pid_IdIqX_obj,
            CURRENT_SCALE,
            VOLTAGE_SCALE,
            TF_REGULATION_RATE,
            FREQUENCY_SCALE,
            fDutyLimit);
    PIDREGDQX_CURRENT_setKp_si(&g_axis.currCtrl.pid_IdIqX_obj, 1.5f);
    PIDREGDQX_CURRENT_setWi_si(&g_axis.currCtrl.pid_IdIqX_obj, 20.0f);
    PIDREGDQX_CURRENT_setOutputLimitsD(&g_axis.currCtrl.pid_IdIqX_obj,
            FIXP30(fDutyLimit * 0.95f), FIXP30(-fDutyLimit * 0.95f));
    PIDREGDQX_CURRENT_setOutputLimitsQ(&g_axis.currCtrl.pid_IdIqX_obj,
            FIXP30(fDutyLimit), FIXP30(-fDutyLimit));
}

static void Speed_Pid_Init(void)
{
    fixp30_t currentLimit_pu = FIXP30((float)PID_MAX_CURRENT / (float)CURRENT_SCALE);
    PIDREG_SPEED_init(&g_axis.speedCtrl.PIDSpeed, CURRENT_SCALE, FREQUENCY_SCALE, SPEED_CONTROL_RATE);
    PIDREG_SPEED_setOutputLimits(&g_axis.speedCtrl.PIDSpeed, currentLimit_pu, -currentLimit_pu);
    PIDREG_SPEED_setKp_si(&g_axis.speedCtrl.PIDSpeed, 0.18);
    PIDREG_SPEED_setKi_si(&g_axis.speedCtrl.PIDSpeed, 0.25);
}

static void Position_Pid_Init(void)
{
    g_axis.posCtrl.positionHandle.Position_Gain = (uint32_t)POSITION_GAIN_FAR_MIN;
    g_axis.posCtrl.positionHandle.Postiion_Div = (uint32_t)(10000U * ENC_COUNTS_PER_REV);
    g_axis.posCtrl.positionHandle.pos_integral = 0.0f;
    g_axis.posCtrl.positionHandle.Position_Ki = POSITION_CAPTURE_KI_NUMERATOR;
    g_axis.posCtrl.positionHandle.Position_Ki_Div = POSITION_CAPTURE_KI_DENOMINATOR;
    g_axis.posCtrl.positionHandle.pos_integral_lim = POSITION_CAPTURE_I_LIMIT_RPM;
    g_axis.posCtrl.positionHandle.pos_correction_prev = 0.0f;
    g_axis.posCtrl.positionHandle.prev_target_error = 0;
    g_axis.posCtrl.positionHandle.prev_error_angle = 0;
    g_axis.posCtrl.positionHandle.Position_Kd = 25U;
    g_axis.posCtrl.positionHandle.Position_Kd_Div = 1000U;
    g_axis.posCtrl.positionHandle.pos_deriv_lim = 10.0f;

    g_axis.posCtrl.traj.vel_max = TRAJ_DEFAULT_VEL_MAX_RPM * ENC_COUNTS_PER_REV / 60000.0f;
    g_axis.posCtrl.traj.acc_max = TRAJ_DEFAULT_ACC_RPMPS * ENC_COUNTS_PER_REV / 60000000.0f;
    g_axis.posCtrl.traj.vel_ref = 0.0f;
    g_axis.posCtrl.traj.acc_ref = 0.0f;
    g_axis.posCtrl.traj.pos_ref = 0.0f;
    g_axis.posCtrl.traj.inited = false;
    g_axis.posCtrl.traj.bEnable = false;
    g_axis.posCtrl.traj.dbg_phase = ' ';
}

void Motor_Control_Init(void)
{
    g_axis.state = AXIS_STATE_UNDEFINED;
    g_axis.bMCBootCompleted = false;

    g_axis.pPWMCHandle = &pwmcHandle;

    Sampling_Init();
    Init_Encoder();
    iqFilter.fAlpha = 0.5f;
    Kalman_Filter_Init(&g_motorSpeedKalmanFilter, KF_R, KF_Q);

    g_axis.posCtrl.hzToStepAngle_pu = FIXP30(FREQUENCY_SCALE / PWM_FREQUENCY);
    g_axis.posCtrl.openLoopAngle_pu = FIXP30(0.0f);
    g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
    g_axis.currCtrl.refIdq.Q = FIXP30(0.1f);
    g_axis.enCtrlMode = CTRL_MODE_OPEN_LOOP;

    g_axis.pPWMCHandle->uCalibCount = 0;

    g_axis.posCtrl.uOffsetAngleRaw = 0U;
    g_axis.posCtrl.uCalibCount = 0;
    g_axis.posCtrl.bCalibFlag = false;
    g_axis.posCtrl.bResetFlag = false;
    g_axis.posCtrl.iAbsRawPos = 0;
    g_axis.posCtrl.iZeroAngle = -1;
    g_axis.posCtrl.uCircle = 0U;
    g_axis.posCtrl.fPosRef = 0.0f;
    g_axis.posCtrl.fSpeedMax = 0.0f;
    g_axis.fRs = 0.0f;
    g_axis.fLs = 0.0f;
    g_axis.fKt = 0.0f;
    g_axis.uPolePairs = (uint8_t)POLE_PAIR_NUM;

    PID_All_Init();

    g_axis.speedCtrl.speedRef_pu = FIXP30(0.0f);
    g_axis.speedCtrl.speedRefRamp_pu = FIXP30(0.0f);
    g_axis.speedCtrl.iqOut_pu = FIXP30(0.0f);
    MC_Set_Speed_Ramp(7.0f);

    g_axis.state = AXIS_STATE_IDLE;
    g_axis.bMCBootCompleted = true;
    g_axis.enCtrlMode = CTRL_MODE_SPEED;

    s_iqRawDisplayA = 0.0f;
    s_iqFilteredDisplayA = 0.0f;
    s_speedCount = 0U;
}

uint16_t FOC_Control(void)
{
    uint16_t uFocCode = MC_NO_ERROR;
    bool bRunSpeedTask = false;

    Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas, NULL);

    s_speedCount++;
    if (s_speedCount >= SPEED_CONTROL_COUNT)
    {
        Calc_Speed(&g_axis.speedCtrl.speedMeas_pu);
#if SPEED_MEAS_INVERT
        g_axis.speedCtrl.speedMeas_pu = -g_axis.speedCtrl.speedMeas_pu;
#endif
        s_speedCount = 0U;
        bRunSpeedTask = true;
    }

    if ((g_axis.enCtrlMode == CTRL_MODE_POSITION) && bRunSpeedTask)
    {
        Position_Control(&g_axis.posCtrl);
        g_axis.speedCtrl.speedRef_pu = g_axis.posCtrl.speedOut_pu;
    }

    if (g_axis.enCtrlMode != CTRL_MODE_TORQUE)
    {
        if (bRunSpeedTask)
        {
            Speed_Control(&g_axis.speedCtrl, g_axis.enCtrlMode);
            g_axis.currCtrl.refIdq.Q = g_axis.speedCtrl.iqOut_pu;
        }
    }
    else
    {
        g_axis.currCtrl.refIdq.Q = FIXP30(0.01f);
    }

    CurrCtrlInput_t currCtrlInput;
    currCtrlInput.Irst_in_pu.R = g_axis.currCtrl.IrstMeas.R;
    currCtrlInput.Irst_in_pu.S = g_axis.currCtrl.IrstMeas.S;
    currCtrlInput.Irst_in_pu.T = g_axis.currCtrl.IrstMeas.T;
    currCtrlInput.Udcbus_in_pu = g_axis.busVoltage;

    uFocCode = Curr_Control(&g_axis.currCtrl, &currCtrlInput);
    return uFocCode;
}

uint16_t Curr_Control(CurrCtrl_t *pCurrCtrl, CurrCtrlInput_t* pCurrCtrlInput)
{
    fixp24_t busVoltageComp = Bus_Voltage_Compensation(pCurrCtrl, pCurrCtrlInput->Udcbus_in_pu);
    PIDREGDQX_CURRENT_setCompensation(&pCurrCtrl->pid_IdIqX_obj, busVoltageComp);

    Duty_Dab_t dutyAB;
    Duty_Drst_t dutyRst;

    fixp30_t anglePark_pu;
    Get_Angle(&anglePark_pu);

    FIXP_CosSin_t cossinPark;
    FIXP30_CosSinPU(anglePark_pu, &cossinPark);
    Clarke_Current(pCurrCtrlInput->Irst_in_pu, &pCurrCtrl->calcIab);

    Park_Current(pCurrCtrl->calcIab, &cossinPark, &pCurrCtrl->calcIdq);

    const float iqRawA = FIXP30_toF(pCurrCtrl->calcIdq.Q) * CURRENT_SCALE;
    CurrentFilter_Update(&iqFilter, iqRawA);
    s_iqRawDisplayA = iqRawA;
    s_iqFilteredDisplayA = iqFilter.fCurr;

    PIDREGDQX_CURRENT_setUiD_pu(&pCurrCtrl->pid_IdIqX_obj, FIXP30(0.0f));
    PIDREGDQX_CURRENT_run(&pCurrCtrl->pid_IdIqX_obj,
            FIXP30(0.0f),
            (pCurrCtrl->refIdq.Q - pCurrCtrl->calcIdq.Q),
            FIXP30(0.0f));

    pCurrCtrl->outIdq.D = FIXP30(0.0f);
    pCurrCtrl->outIdq.Q = PIDREGDQX_CURRENT_getOutQ(&pCurrCtrl->pid_IdIqX_obj);

    Get_Angle(&anglePark_pu);
    FIXP30_CosSinPU(anglePark_pu, &cossinPark);
    Inv_Park_Duty(pCurrCtrl->outIdq, &cossinPark, &dutyAB);
    Inv_Clarke_Duty(dutyAB, &dutyRst);

    return Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);
}

void Speed_Control(SpeedCtrl_t *pSpeedCtrl, ControlMode_En enControlMode)
{
    fixp30_t speedRamp = pSpeedCtrl->speedRamp_pu;
    fixp30_t speedDelta_pu = pSpeedCtrl->speedRef_pu - pSpeedCtrl->speedRefRamp_pu;
    pSpeedCtrl->speedRefRamp_pu += FIXP_sat(speedDelta_pu, speedRamp, -speedRamp);

    if (enControlMode == CTRL_MODE_POSITION)
    {
        pSpeedCtrl->speedRefRamp_pu = pSpeedCtrl->speedRef_pu;
    }

    fixp30_t speedError = pSpeedCtrl->speedRefRamp_pu - pSpeedCtrl->speedMeas_pu;
    fixp30_t IqRef_pu = PIDREG_SPEED_run(&pSpeedCtrl->PIDSpeed, speedError);
    fixp30_t IqRefCircleMax = FIXP30(12.0 / CURRENT_SCALE);

    pSpeedCtrl->iqOut_pu = FIXP_sat(IqRef_pu, IqRefCircleMax, -IqRefCircleMax);
}

void Position_Control(PosCtrl_t *pPosCtrl)
{
    float fOutSpeed = Pid_Position_Run(pPosCtrl);
    float fElectricalFreq = ((float)POLE_PAIR_NUM * fOutSpeed / 60.0f);
    pPosCtrl->speedOut_pu = FIXP30(fElectricalFreq / FREQUENCY_SCALE);
}

fixp30_t Bus_Voltage_Compensation(CurrCtrl_t *pCurrCtrl, const fixp30_t udc_pu)
{
    fixp24_t busVoltageComp = pCurrCtrl->busVoltageComp;

    fixp24_t temp = FIXP(1.0f) - FIXP30_mpy(udc_pu, busVoltageComp);
    fixp24_t factor = FIXP_mpyFIXPscaled(busVoltageComp, &pCurrCtrl->busVoltageFilter);
    busVoltageComp += FIXP_mpy(temp, factor);
    busVoltageComp = FIXP_sat(busVoltageComp, pCurrCtrl->busVoltageCompMax, pCurrCtrl->busVoltageCompMin);
    pCurrCtrl->busVoltageComp = busVoltageComp;

    return busVoltageComp;
}

void Open_Loop_Control(void)
{
    static uint16_t s_openLoopSpeedCount = 0U;

    s_openLoopSpeedCount++;
    if (s_openLoopSpeedCount >= SPEED_CONTROL_COUNT)
    {
        Calc_Speed(&g_axis.speedCtrl.speedMeas_pu);
        s_openLoopSpeedCount = 0U;
    }

    SwitchOn_PWM(g_axis.pPWMCHandle);

    Get_RST_Measurements(g_axis.pPWMCHandle, &g_axis.currCtrl.IrstMeas, NULL);
    Update_Open_Loop_Angle(&g_axis.posCtrl, g_axis.speedCtrl.speedRef_pu);

    fixp30_t anglePark_pu = g_axis.posCtrl.openLoopAngle_pu;
    FIXP_CosSin_t cossinPwm;
    FIXP30_CosSinPU(anglePark_pu, &cossinPwm);

    Duty_Dab_t dutyAb;
    Duty_Drst_t dutyRst;

    g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
    g_axis.currCtrl.refIdq.Q = FIXP30(OPEN_LOOP_IQ_REF_A);

    Inv_Park_Duty(g_axis.currCtrl.refIdq, &cossinPwm, &dutyAb);
    Inv_Clarke_Duty(dutyAb, &dutyRst);
    Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);
}

void Update_Open_Loop_Angle(PosCtrl_t *pHandle, fixp30_t speedRef)
{
    fixp30_t angleStep_pu = FIXP30_mpy(speedRef, pHandle->hzToStepAngle_pu);
    fixp30_t angle_pu = pHandle->openLoopAngle_pu;
    angle_pu += angleStep_pu;
    angle_pu &= (FIXP30(1.0f) - 1);
    pHandle->openLoopAngle_pu = angle_pu;
}

float MotorControl_GetIqRawDisplayA(void)
{
    return s_iqRawDisplayA;
}

float MotorControl_GetIqFilteredDisplayA(void)
{
    return s_iqFilteredDisplayA;
}
