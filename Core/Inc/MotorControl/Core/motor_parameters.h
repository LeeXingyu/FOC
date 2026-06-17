/*
 * motor_parameters.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_
#define INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_

#define POLE_PAIR_NUM                       7
#define PWM_FREQUENCY                       16000
#define REGULATION_EXECUTION_RATE           1

#define TF_REGULATION_RATE                  (uint32_t)((uint32_t)(PWM_FREQUENCY) / (REGULATION_EXECUTION_RATE))
#define BOARD_MAX_MODULATION                (float_t)((100 * 1.15f) / 100.0f)
#define SPEED_CONTROL_RATE                  500
#define SPEED_MEASUREMENT_RATE_HZ           1000.0f
#define SPEED_MEAS_WINDOW_SAMPLES           2U
#define SPEED_CONTROL_COUNT                 32
#define POSITION_CONTROL_COUNT              16

#define RSHUNT                              0.005f
#define ADC_REFERENCE_VOLTAGE               3.3f
#define AMPLIFICATION_GAIN                  40

#define CURRENT_SCALE                       (float_t)(12)
#define VOLTAGE_SCALE                       (float_t)(264)
#define FREQUENCY_SCALE                     (float_t)(1000.0f)

#define MOTOR_ENCODER_REVERSE               1
#define SPEED_MEAS_INVERT                   0
#define TIM_CLOCK_HZ                        170000000U
#define PWM_FREQ_HZ                         16000U
#define HALF_PWM_PERIOD                     (TIM_CLOCK_HZ / (2 * PWM_FREQ_HZ))
#define TICKS_50PCT                         (HALF_PWM_PERIOD / 2)
#define CURR_CALIB_COUNT                    16000
#define ENCODER_CALIB_COUNT                 32000
#define PWM_PERIOD_CYCLES                   10624

#define PID_MAX_CURRENT                     1.5f

#endif /* INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_ */
