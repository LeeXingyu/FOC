/*
 * speed_pos_type.h
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_
#define INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_

#include <stdio.h>
#include <stdint.h>

typedef struct
{
  uint16_t uAngleRaw;			// 归一化编码器原始值
  float fAngle;					// 角度（浮点）
  float fOffsetAngle;			// 偏移角度
  uint16_t uOffsetAngleRaw;		// 偏移编码器原始值
  float fSpeed;                	// M法速度+滤波
  float fSpeedPll;				// PLL测速
  float fSpeedKalman;         	// 卡尔曼滤波

  uint16_t uCircle;				// 当前圈数
} SpeedAngleParam_t;

#endif /* INC_MOTORCONTROL_FBDK_SPEED_POS_TYPE_H_ */
