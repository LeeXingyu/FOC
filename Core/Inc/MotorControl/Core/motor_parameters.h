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
#define SPEED_MEASUREMENT_RATE_HZ           1000.0f
#define SPEED_CONTROL_COUNT                 32
#define SPEED_CONTROL_RATE                  (uint32_t)((uint32_t)(TF_REGULATION_RATE) / (SPEED_CONTROL_COUNT))
#define SPEED_MEAS_WINDOW_SAMPLES           2U
#define POSITION_CONTROL_COUNT              16

/*
 * Speed-loop segmented PI.
 * 1: enable low-speed/high-speed gain switching
 * 0: use the default gains everywhere
 */
#ifndef SPEED_PI_SEGMENTED_ENABLE
#define SPEED_PI_SEGMENTED_ENABLE           1U
#endif

/* Low-speed PI band: use stronger gains below this mechanical speed. */
#define SPEED_PI_LOW_BAND_RPM               40.0f

/* Default gains for 40~100 rpm and above. */
#define SPEED_PI_DEFAULT_KP                 2.62f
#define SPEED_PI_DEFAULT_KI                 1.30f

/* Low-speed gains below 40 rpm. */
#define SPEED_PI_LOW_KP                     7.40f
#define SPEED_PI_LOW_KI                     9.00f

/*
 * CiA 402 compatibility layer over the existing custom CAN transport.
 * Keep enabled by default so the new object mapping can coexist with
 * the legacy command set.
 */
#ifndef APP_USE_CIA402_CAN
#define APP_USE_CIA402_CAN                  0U
#endif

#define CIA402_MODE_NONE                    0
#define CIA402_MODE_PROFILE_POSITION        1
#define CIA402_MODE_PROFILE_VELOCITY        3
#define CIA402_MODE_PROFILE_TORQUE          4
#define CIA402_MODE_CYCLIC_SYNC_POSITION    8
#define CIA402_MODE_CYCLIC_SYNC_VELOCITY    9
#define CIA402_MODE_CYCLIC_SYNC_TORQUE      10

/*
 * Speed measurement mode:
 * 0: current angle-observer / Kalman path
 * 1: PLL observer path
 */
#define SPEED_MEAS_MODE_KALMAN              0U
#define SPEED_MEAS_MODE_PLL                 1U

#ifndef SPEED_MEAS_MODE
#define SPEED_MEAS_MODE                     SPEED_MEAS_MODE_PLL
#endif

/*
 * PLL observer tuning.
 * These values are only used when SPEED_MEAS_USE_PLL_OBSERVER == 1.
 */
#define SPEED_PLL_KP                        1.2f
#define SPEED_PLL_KI                        3.50f
#define SPEED_PLL_MAX_RPM                   3000.0f
#define SPEED_PLL_OUTPUT_LPF_ALPHA          0.04f

/*
 * Low-speed torque compensation.
 * Added to speed-loop Iq output with direction sign following speed command.
 */
#ifndef SPEED_TORQUE_COMP_ENABLE
#define SPEED_TORQUE_COMP_ENABLE            0U
#endif
/*
 * Human-friendly tuning units:
 * - SPEED_TORQUE_COMP_IQ_A: compensation current in ampere
 * - SPEED_TORQUE_COMP_SPEED_RPM: compensation active below this mechanical speed
 */
#define SPEED_TORQUE_COMP_IQ_A              0.2f
#define SPEED_TORQUE_COMP_SPEED_RPM         40.0f

#define SPEED_TORQUE_COMP_IQ_PU             (SPEED_TORQUE_COMP_IQ_A / CURRENT_SCALE)
#define SPEED_TORQUE_COMP_SPEED_PU          (((SPEED_TORQUE_COMP_SPEED_RPM) * (float_t)POLE_PAIR_NUM / 60.0f) / FREQUENCY_SCALE)

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

/* Open-loop startup torque current reference in ampere. */
#define OPEN_LOOP_IQ_REF_A                  0.05f

#endif /* INC_MOTORCONTROL_CORE_MOTOR_PARAMETERS_H_ */
