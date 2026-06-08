/*
 * mc_tasks.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_TASKS_MC_TASKS_H_
#define INC_MOTORCONTROL_TASKS_MC_TASKS_H_

#include "param_identify.h"
#include "mc_interface.h"
#include "cmsis_os.h"

#define MC_HIGH_FREQ_TASK_FLAG     (1U << 0)
#define MC_MEDIUM_FREQ_TASK_FLAG   (1U << 1)

extern osThreadId_t mcHighFreqTaskHandle;
extern osThreadId_t mcMediumFreqTaskHandle;

/**
 * @brief  高频任务（tim1定时器触发16khz+）
 */
void High_Frequency_Task();

/**
 * @brief  中频任务（定时器触发，1khz）
 */
void Medium_Frequency_Task();

void StartHighFrequencyTask(void *argument);
void StartMediumFrequencyTask(void *argument);

/**
 * @brief  状态机转换
 */
void Axis_State_Machine();

void MC_Calib_Init(void);
MC_RetStatus_t MC_Calib_StartChain(void);
MC_RetStatus_t MC_Calib_StartParam(ParamIdStep_t step);
MC_RetStatus_t MC_Calib_StopParam(void);
ParamIdState_t MC_Calib_GetParamState(void);
const ParamIdResult_t *MC_Calib_GetParamResult(void);

#endif /* INC_MOTORCONTROL_TASKS_MC_TASKS_H_ */
