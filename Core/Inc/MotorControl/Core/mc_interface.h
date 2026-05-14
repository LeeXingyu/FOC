/*
 * mc_interface.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_MC_INTERFACE_H_
#define INC_MOTORCONTROL_MC_INTERFACE_H_

#include "mc_type.h"

#define MC_SUCCESS                          ((uint32_t)(0u))
#define MC_FAILED                          	((uint32_t)(1u))

typedef uint32_t MC_RetStatus_t;

/**
  * @brief  电机开始运行
  */
MC_RetStatus_t MC_Start_Motor(void);

/**
  * @brief  电机停止运行
  */
MC_RetStatus_t MC_Stop_Motor(void);

/**
  * @brief  设置控制模式
  */
void MC_Set_Control_Mode(ControlMode_En enControlMode);

/**
  * @brief  电流校准
  */
void MC_Curr_Offset_Cali();

/**
  * @brief  零点校准
  */
void MC_Zero_Calibration();

/**
  * @brief  设置id、iq幅值（开环使用）
  * @param  dutyCycle 归一化幅值
  */
void MC_Set_Duty_Cycle(Duty_Ddq_t dutyCycle);

/**
  * @brief  设置电气频率得变化率
  * @param  fRamp 变化率（hz/s）
  */
void MC_Set_Speed_Ramp(float fRamp);

/**
  * @brief  设置速度环kp
  * @param  fKp 速度环kp
  */
void MC_Set_Speed_Kp(float fKp);

/**
  * @brief  设置速度环ki
  * @param  fKi 速度环ki
  */
void MC_Set_Speed_Ki(float fKi);

/**
  * @brief  设置目标速度
  * @param  fRefSpeed 目标速度（rpm）
  */
void MC_Set_Speed_Reference(float fRefSpeed);

#endif /* INC_MOTORCONTROL_MC_INTERFACE_H_ */
