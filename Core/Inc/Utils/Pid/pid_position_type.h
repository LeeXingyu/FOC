/*
 * pid_position_type.h
 *
 *  Created on: May 9, 2026
 *      Author: Administrator
 */

#ifndef INC_UTILS_PID_PID_POSITION_TYPE_H_
#define INC_UTILS_PID_PID_POSITION_TYPE_H_

#include "encoder.h"

/* T型轨迹规划相关换算 */
#define ENC_COUNTS_PER_REV   ENCODER_COUNT

/* 转换: RPM -> counts/ms */
#define RPM_TO_COUNTS_MS(rpm)    ((rpm) * ENC_COUNTS_PER_REV / 60000.0f)
/* 转换: RPM/s -> (counts/ms)/调用 */
#define RPMPS_TO_ACCEL(a)        ((a) * ENC_COUNTS_PER_REV / 60000000.0f)
#define COUNTS_MS_TO_RPM(v)      ((v) * 60000.0f / ENC_COUNTS_PER_REV)
#define TRAJ_DEFAULT_VEL_MAX_RPM  (250.0f)
#define TRAJ_DEFAULT_ACC_RPMPS    (100.0f)

typedef struct {
    float pos_ref;    /* planned position reference   [encoder counts] */
    float vel_ref;    /* planned velocity             [counts/ms] */
    float acc_ref;    /* current accel                [(counts/ms)/call] */
    float vel_max;    /* configurable max velocity    [counts/ms] */
    float acc_max;    /* configurable max accel       [(counts/ms)/call] */
    float j_max;      /* configurable max jerk        [(counts/ms)/call^2] */
    bool  inited;     /* lazy-init flag */
    char  dbg_phase;  /* debug: 'A'=Accel 'C'=Cruise 'D'=Decel ' '=Idle */
    uint8_t traj_mode;/* TRAJ_MODE_T or TRAJ_MODE_S */
    bool  bEnable;    /* true=enable T-curve, false=direct pass-through */
} TrajPlanner_t;

typedef struct
{
    uint32_t Position_Gain;      /* Position Kp gain */
    uint32_t Postiion_Div;       /* Position Kp divider */

    /* ---- Position I term ---- */
    float    pos_integral;       /* I term accumulator [RPM] */
    uint32_t Position_Ki;        /* Ki numerator */
    uint32_t Position_Ki_Div;    /* Ki denominator */
    float    pos_integral_lim;   /* integral limit [RPM] */

    float    pos_correction_prev;/* Previous rate-limited correction [RPM] */
    int64_t  prev_target_error;  /* Previous final-target error [counts] */

    /* ---- Position D term ---- */
    int32_t  prev_error_angle;   /* Previous error angle [counts] */
    uint32_t Position_Kd;        /* Kd numerator */
    uint32_t Position_Kd_Div;    /* Kd denominator */
    float    pos_deriv_lim;      /* derivative output limit [RPM] */

} Position_Handle_t;


#endif /* INC_UTILS_PID_PID_POSITION_TYPE_H_ */
