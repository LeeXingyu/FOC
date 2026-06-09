/*
 * speed_pos_fbdk.h
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_FBDK_SPEED_POS_FBDK_H_
#define INC_MOTORCONTROL_FBDK_SPEED_POS_FBDK_H_


#include <stdint.h>
#include "Utils/Fixp/fixpmath.h"

/**
 * @brief  圈数量计算
 */
void Circle_Update();

/**
  * @brief  计算并获取电气速度
  *
  * @param  pSpeed: 电机速度
 */
void Calc_Speed(fixp30_t *pSpeed);

/**
  * @brief  获取电气速度
  *
  * @param  pSpeed: 电机速度
 */
void Get_Speed(fixp30_t *pSpeed);

/**
  * @brief  获取电气角度
  *
  * @param  pAngle: 电气角度
 */
void Get_Angle(fixp30_t *pAngle);

/**
  * @brief  获取编码器角度原始值
  *
 */
uint16_t Get_Angle_Raw();
uint32_t Get_Angle_RawNative();
uint32_t Get_Angle_CountNative();

/**
 * @brief  M法测速+低通滤波
 */
void Sensor_Update();

/**
 * @brief  卡尔曼滤波测速
 */
void Sensor_Update_Kalman();

/**
 * @brief  PLL锁相环测速
 */
void Sensor_Update_PLL();

/**
 * @brief  重置速度/圈数估算历史，避免启动时旧值影响控制。
 */
void SpeedPos_ResetEstimator(void);


#endif /* INC_MOTORCONTROL_FBDK_SPEED_POS_FBDK_H_ */
