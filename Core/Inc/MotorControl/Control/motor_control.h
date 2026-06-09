/*
 * motor_control.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_CORE_MOTOR_CONTROL_H_
#define INC_MOTORCONTROL_CORE_MOTOR_CONTROL_H_

#include "MotorControl/Core/mc_type.h"

/**
  * @brief  电机控制初始化
  */
void Motor_Control_Init(void);

/**
  * @brief  foc控制
  */
uint16_t FOC_Control(void);

/**
  * @brief  电流控制（电流环）
  */
void Curr_Control(CurrCtrl_t *pCurrCtrl, CurrCtrlInput_t* pCurrCtrlInput);

/**
  * @brief  速度控制（速度环）
  */
void Speed_Control(SpeedCtrl_t *pSpeedCtrl, ControlMode_En enControlMode);

/**
  * @brief  位置控制（位置环）
  * @param  pPosCtrl	  	位置控制句柄
  */
void Position_Control(PosCtrl_t *pPosCtrl);

fixp30_t Bus_Voltage_Compensation(CurrCtrl_t *pCurrCtrl, const fixp30_t udc_pu);

/**
 * @brief  开环旋转测试, V/F 控制（电压随频率同步变化）
 */
void Open_Loop_Control();

#endif /* INC_MOTORCONTROL_CORE_MOTOR_CONTROL_H_ */
