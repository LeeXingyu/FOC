/*
 * mc_tasks.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_TASKS_MC_TASKS_H_
#define INC_MOTORCONTROL_TASKS_MC_TASKS_H_


/**
 * @brief  高频任务（tim1定时器触发16khz+）
 */
void High_Frequency_Task();

/**
 * @brief  中频任务（定时器触发，1khz）
 */
void Medium_Frequency_Task();

/**
 * @brief  状态机转换
 */
void Axis_State_Machine();


#endif /* INC_MOTORCONTROL_TASKS_MC_TASKS_H_ */
