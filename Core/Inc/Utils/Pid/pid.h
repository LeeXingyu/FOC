/*
 * pid.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_UTILS_PID_H_
#define INC_UTILS_PID_H_

#include <stdint.h>
#include <stdbool.h>
#include "fixpmath.h"

typedef enum
{
    PID_MODE_P = 0,
    PID_MODE_PI,
    PID_MODE_PID
} PID_Mode_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float dt;      // 控制周期(s)
    float tau;     // 微分低通时间常数(s), 推荐 dt~10*dt
    PID_Mode_t mode;
} PID_Param_t;

typedef struct
{
    float outMin;
    float outMax;
    float iMin;
    float iMax;
} PID_Limit_t;

typedef struct
{
    float integral;
    float prevMeas;
    float dFilt;
    float prevOut;
    bool initialized;
} PID_State_t;

typedef struct
{
    PID_Param_t param;
    PID_Limit_t limit;
    PID_State_t state;

    fixp30_t  fp_Kp;
	fixp30_t  fp_Ki_dt;    // ki * dt 预乘
	fixp30_t  fp_outMax;
	fixp30_t  fp_outMin;
	fixp30_t  fp_intMax;
	fixp30_t  fp_intMin;
	fixp30_t  fp_integral;
} PID_Handle_t;

/* 初始化/重置 */

/*
 * 初始化
 * */
void PID_Init(PID_Handle_t *pid, const PID_Param_t *param, const PID_Limit_t *limit);

/*
 * 重置
 * */
void PID_Reset(PID_Handle_t *pid, float outInit_pu);

void PID_SetGains(PID_Handle_t *pid, float kp, float ki, float kd);

void PID_SetLimits(PID_Handle_t *pid, float outMin, float outMax, float iMin, float iMax);

/* 核心运行: 输入输出均为 pu */
float PID_Run_PU(PID_Handle_t *pid, float ref_pu, float meas_pu, float ff_pu);

/* fixp30 包装接口: 便于直接接入你现有框架 */
fixp30_t PID_Run_FIXP30(PID_Handle_t *pid, fixp30_t ref_pu, fixp30_t meas_pu, fixp30_t ff_pu);

#endif /* INC_UTILS_PID_H_ */
