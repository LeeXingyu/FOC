/*
 * motor_parameters.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_
#define INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_

#define POLE_PAIR_NUM           			14 					/* 电机极对数 */
#define PWM_FREQUENCY						16000				/* pwm 频率 */
#define REGULATION_EXECUTION_RATE           1

#define TF_REGULATION_RATE                  (uint32_t)((uint32_t)(PWM_FREQUENCY) / (REGULATION_EXECUTION_RATE))
#define BOARD_MAX_MODULATION                (float_t)((100 * 1.15f) /100.0f)
#define SPEED_CONTROL_RATE					1000				/* 速度环频率 */

#define RSHUNT                              0.005f				/* 采样电阻 */
#define ADC_REFERENCE_VOLTAGE				3.3f				/* ADC参考电压 */
#define AMPLIFICATION_GAIN					40					/* 电流采样放大电路增益 */

#define CURRENT_SCALE                       (float_t)(12) 		/* 电流归一化计算比例尺 */
#define VOLTAGE_SCALE                       (float_t)(264) 		/* 电压归一化计算比例尺 */
#define FREQUENCY_SCALE                     (float_t)(1000.0f) 	/* 电机转动频率归一化计算比例尺 */

#define TIM_CLOCK_HZ        170000000U
#define PWM_FREQ_HZ         16000U
#define HALF_PWM_PERIOD     (TIM_CLOCK_HZ / (2 * PWM_FREQ_HZ))
#define TICKS_50PCT         (HALF_PWM_PERIOD / 2) 				// ???是否有问题，待验证

#define CURR_CALIB_COUNT	16000								// 电流校准计数（1s）
#define ENCODER_CALIB_COUNT	32000								// 编码器校准计数（500ms）
#define PWM_PERIOD_CYCLES	10624


#define PID_MAX_CURRENT		30									/* 速度环最大电流 */

#endif /* INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_ */
