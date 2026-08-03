/*
 * curr_autotune.h
 *
 * 定子电阻及 dq 轴电感离线辨识。
 */

#ifndef INC_MOTORCONTROL_CONTROL_CURR_AUTOTUNE_H_
#define INC_MOTORCONTROL_CONTROL_CURR_AUTOTUNE_H_

#include <stdbool.h>
#include <stdint.h>

#include "fixpmath_types.h"
#include "motor_parameters.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Rs 辨识：两相直流电流缓慢爬升到目标值，再在正、负方向采样稳态 U/I。 */
#define AVG_SAMPLE_COUNT                 (50U)    /* 每个方向的稳态平均样本数。 */
#define AUTOTUNE_RS_RATE_DIVIDER         (16U)    /* Rs 电流调节频率：PWM_FREQUENCY / 16。 */
#define AUTOTUNE_RS_TARGET_CURRENT_A     (0.80f)  /* 通用测试电流，兼顾低阻航模和关节电机。 */
#define AUTOTUNE_RS_CURRENT_TOL_A        (0.08f)  /* 进入稳态判断前允许的目标电流误差。 */
#define AUTOTUNE_RS_STABLE_TICKS         (200U)   /* 目标电流连续保持时间，按 Rs 调节节拍计。 */
#define AUTOTUNE_RS_MIN_SAMPLE_CURRENT_A (0.10f)  /* Allow high-resistance motors to be measured at lower current. */
#define AUTOTUNE_RS_DUTY_KI              (1.0e-3f)/* Rs 电流调节积分增益。 */
#define AUTOTUNE_RS_MAX_DUTY_STEP        (0.00025f)/* 单次调节的最大占空比变化。 */
#define AUTOTUNE_RS_MAX_DUTY             (0.20f)  /* Voltage headroom for high-resistance gimbal motors. */
#define AUTOTUNE_RS_LOCK_TICKS           (100U)   /* 正向电流预锁定时间，按 Rs 调节节拍计。 */
#define AUTOTUNE_RS_STAGE_MAX_TICKS      (3000U)  /* 单方向最长辨识时间，防止无法收敛时卡死。 */
#define AUTOTUNE_MAX_PHASE_CURRENT_A     (2.50f)  /* 所有辨识阶段逐 PWM 周期检查的相电流上限。 */

/* Ld/Lq 辨识：以完整 PWM 中断频率运行，转子保持在固定电角度。 */
#define AUTOTUNE_ID_BIAS_A               (0.50f)
#define AUTOTUNE_MIN_ID_BIAS_A           (0.10f)  /* Minimum useful rotor-lock current. */
#define AUTOTUNE_ALIGN_DUTY_UTILIZATION  (0.70f)  /* Reserve duty headroom for regulation and injection. */
#define AUTOTUNE_ALIGN_DUTY_KI           (1.0e-4f) 						/* At 0.5 A error, ramps at half the proven-safe probe step. */
#define AUTOTUNE_ALIGN_MAX_DUTY_STEP     (1.0e-4f)             /* Bound one-cycle correction under abnormal feedback. */
#define AUTOTUNE_ALIGN_MIN_TICKS         (PWM_FREQUENCY / 2U) 			/* 最短对齐时间。 */
#define AUTOTUNE_ALIGN_MAX_TICKS         (PWM_FREQUENCY * 2U) 			/* Allow settling after the current first reaches its target. */
#define AUTOTUNE_ALIGN_STABLE_TICKS      (PWM_FREQUENCY / 16U) 			/* 必须连续稳定的周期数。 */
#define AUTOTUNE_ALIGN_CURRENT_TOL_A     (0.075f) 					/* Covers one effective PWM step around the 0.5 A target. */
#define AUTOTUNE_ALIGN_IQ_TOL_A          (0.05f)  						/* 对齐期间允许的 Iq。 */
#define AUTOTUNE_ALIGN_ENTRY_CURRENT_A   (0.15f)  /* Rs 反向电流衰减到此值后才开始对齐。 */
#define AUTOTUNE_ALIGN_ENTRY_STABLE_TICKS (32U)   /* 连续低电流确认，过滤关断后的采样瞬态。 */
#define AUTOTUNE_ALIGN_ENTRY_MAX_TICKS   (PWM_FREQUENCY / 20U) /* 最长等待 50 ms。 */
#define AUTOTUNE_ALIGN_PROBE_DUTY        (0.0005f)/* 用小电压判断 dq 输出与电流反馈的极性。 */
#define AUTOTUNE_ALIGN_PROBE_MAX_DUTY    (0.1000f)  /* Covers dead time and high-resistance motor voltage demand. */
#define AUTOTUNE_ALIGN_PROBE_MAX_TICKS   (PWM_FREQUENCY / 20U) /* Ramp window: 50 ms at the configured PWM rate. */
#define AUTOTUNE_ALIGN_PROBE_DUTY_STEP   \
    ((AUTOTUNE_ALIGN_PROBE_MAX_DUTY - AUTOTUNE_ALIGN_PROBE_DUTY) / \
     (float)(AUTOTUNE_ALIGN_PROBE_MAX_TICKS - 1U))
#define AUTOTUNE_ALIGN_PROBE_MIN_CURRENT_A (0.025f) /* Minimum signal used for polarity confirmation. */
#define AUTOTUNE_ALIGN_PROBE_SIGN_TICKS  (16U)     /* Consecutive equal-sign samples reject current noise. */
#define AUTOTUNE_ALIGN_MAX_DUTY          (0.30f)  						/* High-R motors need more voltage to establish Id. */
#define AUTOTUNE_LQ_SETTLE_TICKS         (PWM_FREQUENCY / 10U) 			/* Ld 到 Lq 的稳定等待时间。 */
#define AUTOTUNE_RECOVERY_TICKS          (4U)    						/* 辨识结束后等待的干净 PWM 更新数。 */
#define AUTOTUNE_LD_INJECT_HALF_TICKS    (8U)    /* 16 kHz PWM 时为 1 kHz，限制低电感电机纹波。 */
#define AUTOTUNE_LQ_INJECT_HALF_TICKS    (8U)    /* q 轴同样使用 1 kHz，减小交变转矩。 */
#define AUTOTUNE_INJECT_BLANK_TICKS      (2U)   						/* 极性切换后忽略的瞬态周期数。 */
#define AUTOTUNE_ADAPT_MAX_CYCLES        (20U)   /* 幅值自适应允许的最大完整周期数。 */
#define AUTOTUNE_ADAPT_STABLE_CYCLES     (2U)    /* Ipp 连续落入目标窗口后才开始正式测量。 */
#define AUTOTUNE_MEASURE_CYCLES          (32U)  						/* Ld/Lq 正式测量的完整周期数。 */
#define AUTOTUNE_MEASURE_MAX_CYCLES      (64U)   /* Extend only when motion rejection leaves too few samples. */
#define AUTOTUNE_MAX_ESTIMATES           (AUTOTUNE_MEASURE_MAX_CYCLES * 2U) /* 每半周期一个样本。 */

#define AUTOTUNE_INITIAL_HF_DUTY         (0.0005f)/* 双轴自适应的低风险初始注入占空比。 */
#define AUTOTUNE_MIN_HF_DUTY             (0.0005f)/* 约对应当前定时器的一个有效比较计数。 */
#define AUTOTUNE_MAX_HF_DUTY             (0.300f) /* High-inductance motors need more injection voltage. */
#define AUTOTUNE_TARGET_CURRENT_PP_A     (0.50f)  /* 自适应希望达到的轴电流峰峰值。 */
#define AUTOTUNE_MIN_CURRENT_PP_A        (0.25f)  /* 可接受的 Ipp 下限。 */
#define AUTOTUNE_MAX_CURRENT_PP_A        (0.75f)  /* 可接受的 Ipp 上限。 */
#define AUTOTUNE_MAX_INJECT_CURRENT_A    (1.50f)  						/* Ld/Lq 注入交流电流保护阈值。 */
#define AUTOTUNE_SOFT_ELEC_DRIFT_DEG     (2.0f)   /* Physical floor across the confirmed motion windows. */
#define AUTOTUNE_ENCODER_NOISE_SAMPLES   (64U)    /* Stable-angle samples used for the robust noise estimate. */
#define AUTOTUNE_ENCODER_NOISE_SIGMA     (4.0f)   /* MAD-derived confidence margin for encoder quantization/noise. */
#define AUTOTUNE_NOISE_MAX_ELEC_DEG      (1.0f)   /* Do not learn real rotor motion as encoder noise. */
#define AUTOTUNE_MOTION_WINDOW_TICKS     ((PWM_FREQUENCY + 999U) / 1000U) /* Approximately 1 ms. */
#define AUTOTUNE_MOTION_CONFIRM_WINDOWS  (3U)     /* Consecutive moving windows required to abort. */
#define AUTOTUNE_REBASE_STABLE_WINDOWS   (2U)     /* Stable windows required before accepting a settling shift. */
#define AUTOTUNE_MAX_REBASE_ELEC_DEG     (20.0f)  /* Maximum total settling displacement from alignment. */
#define AUTOTUNE_ENCODER_MAX_STEP_DEG    (10.0f)  /* Larger single-sample jumps are treated as SPI glitches. */
#define AUTOTUNE_ENCODER_GLITCH_TICKS    (4U)     /* Persistent invalid jumps still stop identification. */
#define AUTOTUNE_LD_SEGMENT_DRIFT_DEG    (0.25f)  						/* Ld 单半周期允许的转子漂移。 */
#define AUTOTUNE_LQ_SEGMENT_DRIFT_DEG    (0.50f)  						/* Lq 单半周期允许的转子漂移。 */

#define AUTOTUNE_MIN_RS_OHM              (0.0001f) 						/* Rs 结果有效范围。 */
#define AUTOTUNE_MAX_RS_OHM              (50.0f)
#define AUTOTUNE_MIN_L_H                 (1.0e-6f) 						/* Ld/Lq 结果有效范围。 */
#define AUTOTUNE_MAX_L_H                 (10.0e-3f)

/* 离线辨识状态机：依次完成 Rs/Ld/Lq、电流 PI 写入和恢复。 */
typedef enum
{
    CURR_AUTOTUNE_IDLE = 0,
    CURR_AUTOTUNE_RS_ROTOR_LOCK, 	/* 两相直流预锁定。 */
    CURR_AUTOTUNE_RS_POS,         	/* Rs 正向稳态采样。 */
    CURR_AUTOTUNE_RS_NEG,         	/* Rs 反向稳态采样。 */
    CURR_AUTOTUNE_L_ALIGN,        	/* 固定电角度并建立 Id 偏置。 */
    CURR_AUTOTUNE_LD_ADAPT,       	/* 选择 Ld 注入幅值。 */
    CURR_AUTOTUNE_LD_MEASURE,     	/* 正式测量 Ld。 */
    CURR_AUTOTUNE_LQ_SETTLE,      	/* Ld 到 Lq 的过渡稳定。 */
    CURR_AUTOTUNE_LQ_ADAPT,       	/* 选择 Lq 注入幅值。 */
    CURR_AUTOTUNE_LQ_MEASURE,     	/* 正式测量 Lq。 */
    CURR_AUTOTUNE_VALIDATE,       	/* 把结果写入运行时参数。 */
    CURR_AUTOTUNE_FINISH,         	/* 清零输出并更新电流环参数。 */
    CURR_AUTOTUNE_RECOVER,        	/* 清除定时器状态后恢复运行。 */
    CURR_AUTOTUNE_FAULT           	/* 保护或结果校验失败。 */
} CurrAutoTuneState_t;

/* 对外报告的辨识失败原因。 */
typedef enum
{
    CURR_AUTOTUNE_ERROR_NONE = 0,
    CURR_AUTOTUNE_ERROR_RS_INVALID,    /* Rs 分母或结果范围无效。 */
    CURR_AUTOTUNE_ERROR_BIAS_CURRENT,  /* Id 偏置无法稳定。 */
    CURR_AUTOTUNE_ERROR_OVERCURRENT,   /* 电流超过辨识保护阈值。 */
    CURR_AUTOTUNE_ERROR_ROTOR_MOVED,   /* 转子角度漂移超限。 */
    CURR_AUTOTUNE_ERROR_LD_INVALID,    /* Ld 样本或统计结果无效。 */
    CURR_AUTOTUNE_ERROR_LQ_INVALID     /* Lq 样本或统计结果无效。 */
} CurrAutoTuneError_t;

typedef struct
{
    CurrAutoTuneState_t state;
    CurrAutoTuneError_t error;

    uint32_t stable_cnt;
    float U_plus;
    float I_plus;
    float U_minus;
    float I_minus;
    float Rs;

    uint8_t bAveraging;
    uint16_t avg_cnt;
    float sum_U;
    float sum_I;

    float Ld;
    float Lq;
    float ldDutyAmplitude;
    float lqDutyAmplitude;
    float idTargetA;
    float idBiasA;
    fixp30_t lockAnglePu;
} RsIdent_t;

extern RsIdent_t g_rs_ident;

/**
 * @brief 开始一次新的电气参数离线辨识。
 * @note 该函数只复位辨识状态并打印配置，实际状态机由 Handle 驱动。
 */
void CurrAutoTune_Start(void);

/**
 * @brief 中止当前辨识并关闭 PWM。
 * @note 可从后台任务或用户取消命令中调用。
 */
void CurrAutoTune_Abort(void);

/**
 * @brief 执行一次辨识状态机服务。
 * @note 应在 PWM/FOC 高频任务中周期调用；每次调用只推进当前阶段一步。
 */
void CurrAutoTune_Handle(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTORCONTROL_CONTROL_CURR_AUTOTUNE_H_ */
