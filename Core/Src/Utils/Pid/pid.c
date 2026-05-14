/*
 * pid.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 *  des: pid函数
 */

#include "pid.h"
#include <string.h>

static float clampf(float x, float minVal, float maxVal)
{
    if (x > maxVal) return maxVal;
    if (x < minVal) return minVal;
    return x;
}

void PID_Init(PID_Handle_t *pid, const PID_Param_t *param, const PID_Limit_t *limit)
{
    pid->param = *param;
    pid->limit = *limit;
    memset(&pid->state, 0, sizeof(pid->state));

    // 预计算 fixp30_t 增益，运行时不再做 float 运算
	pid->fp_Kp       = FIXP30(param->kp);
	pid->fp_Ki_dt    = FIXP30(param->ki * param->dt);
	pid->fp_outMax   = FIXP30(limit->outMax);
	pid->fp_outMin   = FIXP30(limit->outMin);
	pid->fp_intMax   = FIXP30(limit->iMax);
	pid->fp_intMin   = FIXP30(limit->iMin);
	pid->fp_integral = 0;
}

void PID_Reinit(PID_Handle_t *pid, const PID_Param_t *param)
{
    // 预计算 fixp30_t 增益，运行时不再做 float 运算
	pid->fp_Kp       = FIXP30(param->kp);
	pid->fp_Ki_dt    = FIXP30(param->ki * param->dt);
}

void PID_Reset(PID_Handle_t *pid, float outInit_pu)
{
    pid->state.integral = 0.0f;
    pid->state.prevMeas = 0.0f;
    pid->state.dFilt = 0.0f;
    pid->state.prevOut = clampf(outInit_pu, pid->limit.outMin, pid->limit.outMax);
    pid->state.initialized = false;
    pid->fp_integral = 0;
}

void PID_SetGains(PID_Handle_t *pid, float kp, float ki, float kd)
{
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
    pid->fp_Kp    = FIXP30(kp);
    pid->fp_Ki_dt = FIXP30(ki * pid->param.dt);
    pid->fp_integral = 0;
}

void PID_SetLimits(PID_Handle_t *pid, float outMin, float outMax, float iMin, float iMax)
{
    pid->limit.outMin = outMin;
    pid->limit.outMax = outMax;
    pid->limit.iMin = iMin;
    pid->limit.iMax = iMax;
    pid->fp_outMax = FIXP30(outMax);
	pid->fp_outMin = FIXP30(outMin);
	pid->fp_intMax = FIXP30(iMax);
	pid->fp_intMin = FIXP30(iMin);
}

fixp30_t PID_Run_FIXP30(PID_Handle_t *pid, fixp30_t ref_pu, fixp30_t meas_pu, fixp30_t ff_pu)
{
	fixp30_t error = ref_pu - meas_pu;

	// 积分更新 + 钳位抗饱和
	pid->fp_integral += FIXP30_mpy(pid->fp_Ki_dt, error);
	if      (pid->fp_integral > pid->fp_intMax) pid->fp_integral = pid->fp_intMax;
	else if (pid->fp_integral < pid->fp_intMin) pid->fp_integral = pid->fp_intMin;

	// P + I + 前馈
	fixp30_t out = FIXP30_mpy(pid->fp_Kp, error) + pid->fp_integral + ff_pu;

	// 输出钳位
	if      (out > pid->fp_outMax) out = pid->fp_outMax;
	else if (out < pid->fp_outMin) out = pid->fp_outMin;

	return out;
}



