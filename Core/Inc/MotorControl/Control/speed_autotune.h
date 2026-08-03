/*
 * speed_autotune.h
 *
 * 平滑离线速度环辨识与 PI 参数验证。
 */

#ifndef INC_MOTORCONTROL_CONTROL_SPEED_AUTOTUNE_H_
#define INC_MOTORCONTROL_CONTROL_SPEED_AUTOTUNE_H_  /* 速度辨识头文件包含保护宏。 */

#include <stdbool.h>
#include <stdint.h>

#include "motor_parameters.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 离线速度环辨识流程的公共配置。
 *
 * 辨识运行期间由本模块独占 Iq 电流指令。流程先搜索可用的激励幅值，再辨识局部
 * “电流到速度”对象，依据模型计算 PI，最后通过受限的双向运动验证 Kp。只有 Kp
 * 验证通过且电机最终稳定停止后，结果才允许标记为有效。
 *
 * `satune <current_A>` 指定的是整个实验允许使用的电流绝对上限，并不是立即施加的
 * 测试电流。自适应正弦从较小的初始电流开始，只有相干速度响应不足时才逐级增大。
 */
#define SPEED_AUTOTUNE_DEFAULT_CURRENT_A            (0.20f)  /* 未指定时采用的测试电流上限，A。 */
#define SPEED_AUTOTUNE_MIN_CURRENT_A                (0.08f)  /* 允许配置的最小测试电流上限，A。 */
#define SPEED_AUTOTUNE_MAX_CURRENT_A                (0.50f)  /* 允许配置的最大测试电流上限，A。 */
#define SPEED_AUTOTUNE_MAX_PHASE_CURRENT_A          (1.00f)  /* 任一相电流的独立硬保护门限，A。 */

/* 独立于模型计算的机械安全限制和阶段时序限制。 */
#define SPEED_AUTOTUNE_MAX_RPM                      (60.0f)  /* 实验允许的最大机械转速，rpm。 */
#define SPEED_AUTOTUNE_MAX_TRAVEL_REV               (1.00f)  /* 相对起点允许的最大累计位移，圈。 */
#define SPEED_AUTOTUNE_GUARD_WINDOW_MS              (8U)     /* 独立位置差分测速窗口长度，ms。 */
#define SPEED_AUTOTUNE_SETTLE_MS                    (300U)   /* 开始注入前的零电流静置时间，ms。 */
#define SPEED_AUTOTUNE_STOP_STABLE_MS               (150U)   /* 判定停车所需的连续稳定时间，ms。 */
#define SPEED_AUTOTUNE_RECOVERY_HF_TICKS            (4U)     /* 重新开启 PWM 前等待的高频调用数。 */

/*
 * 先使用平滑自适应单正弦选择电流幅值，再执行相干三音 FRF 辨识。
 *
 * 0.5 Hz 有意设置得明显低于电流环辨识得到的带宽，使测得响应主要反映机械对象而
 * 不是电流环动态。电流幅值只在正弦过零点改变，因此自适应过程不会产生转矩阶跃。
 */
#define SPEED_AUTOTUNE_SINE_FREQUENCY_HZ            (0.50f)   /* 自适应单正弦及三音基波频率，Hz。 */
#define SPEED_AUTOTUNE_MULTISINE_TONE_COUNT          (3U)     /* 同时注入并同步解调的音调数量。 */
#define SPEED_AUTOTUNE_MULTISINE_PER_TONE_RATIO      (0.48f)  /* 每个音调相对选定电流的幅值比例。 */
#define SPEED_AUTOTUNE_MULTISINE_RESPONSE_RETENTION  (0.50f)  /* 三音阶段最低保留响应比例。 */
#define SPEED_AUTOTUNE_MULTISINE_MIN_VALID_TONES     (2U)     /* 建模所需的最少有效音调数量。 */
#define SPEED_AUTOTUNE_MULTISINE_MIN_IQ_AMPLITUDE_A  (0.005f) /* 单音调最小实测 Iq 幅值，A。 */
#define SPEED_AUTOTUNE_MULTISINE_MIN_SPEED_RPM       (0.005f) /* 单音调最小机械速度幅值，rpm。 */
#define SPEED_AUTOTUNE_MULTISINE_MIN_TRACKING_RATIO  (0.10f)  /* 单音调最小 Iq 跟踪比例。 */
#define SPEED_AUTOTUNE_MULTISINE_MIN_PHASE_DEG       (5.0f)   /* 有效速度/Iq 相位下限，度。 */
#define SPEED_AUTOTUNE_MULTISINE_MAX_PHASE_DEG       (160.0f) /* 有效速度/Iq 相位上限，度。 */
#define SPEED_AUTOTUNE_SINE_INITIAL_CURRENT_A       (0.020f)  /* 自适应搜索起始正弦峰值电流，A。 */
#define SPEED_AUTOTUNE_SINE_CURRENT_STEP_RATIO      (1.50f)   /* 响应不足时的电流幅值递增倍率。 */
/* 仅用于归一化回中 P 阻尼增益的参考速度，不是实际闭环速度目标。 */
#define SPEED_AUTOTUNE_SINE_TARGET_AMPLITUDE_RPM    (6.0f)   /* 回中 P 阻尼归一化参考速度，rpm。 */
/* 退出自适应电流搜索前，速度基波幅值必须稳定达到该门限。 */
#define SPEED_AUTOTUNE_SINE_ADAPT_TARGET_RPM        (4.0f)   /* 常规对象要求的速度基波幅值，rpm。 */
/*
 * 低机动性对象可能无法达到上面的常规响应门限。只有调用方授权的电流上限已经完整
 * 测试一个周期后，才允许使用下面的低响应门限；普通电机仍必须达到 4 rpm 门限。
 */
#define SPEED_AUTOTUNE_CEILING_MIN_RESPONSE_RPM     (0.10f)  /* 电流上限下允许的最低回退响应，rpm。 */
/*
 * 机动性定义为相干速度基波幅值除以施加的正弦电流幅值，单位为机械 rpm/A。
 * lowMobilityScore 按下式计算并限制到 [0, 1]：
 *
 *   score = (HIGH_RPM_PER_A - measured_rpm_per_A)
 *           / (HIGH_RPM_PER_A - LOW_RPM_PER_A)
 *
 * 因此 score=0 表示对象机动性高，score=1 表示对象主要受惯量、折算负载或摩擦
 * 限制。本模块所有按对象特性插值的参数都遵循这个方向。
 */
#define SPEED_AUTOTUNE_MOBILITY_LOW_RPM_PER_A        (2.0f)  /* score=1 对应的低机动性端点，rpm/A。 */
#define SPEED_AUTOTUNE_MOBILITY_HIGH_RPM_PER_A       (20.0f) /* score=0 对应的高机动性端点，rpm/A。 */
/* 每个 Iq 幅值切换后丢弃的完整周期数，用于排除启动瞬态。 */
#define SPEED_AUTOTUNE_SINE_ADAPT_DISCARD_CYCLES    (1U)     /* 幅值切换后丢弃的过渡周期数。 */
#define SPEED_AUTOTUNE_SINE_CENTER_CURRENT_RATIO    (0.50f)  /* 回中阻尼在参考速度下占电流比例。 */
#define SPEED_AUTOTUNE_SINE_CENTER_GAIN_RAMP_MS     (500U)   /* 回中 P 增益渐变时间，ms。 */
#define SPEED_AUTOTUNE_SINE_MAX_ADAPT_CYCLES        (8U)     /* 中间幅值允许评价的最大周期数。 */
#define SPEED_AUTOTUNE_SINE_SETTLE_CYCLES           (1U)     /* 三音淡入后等待的基波周期数。 */
#define SPEED_AUTOTUNE_SINE_MEASURE_CYCLES          (4U)     /* 三音同步解调的基波测量周期数。 */
#define SPEED_AUTOTUNE_SINE_BRAKE_TIMEOUT_MS        (4000U)  /* 三音测量后停车的最长时间，ms。 */
#define SPEED_AUTOTUNE_SINE_STOP_RPM                (1.00f)  /* 三音制动停车窗口半宽，rpm。 */
#define SPEED_AUTOTUNE_SINE_STOP_RESET_RPM          (2.00f)  /* 超过该转速时重置停车计时，rpm。 */

/* 同步解调结果的质量门限。 */
#define SPEED_AUTOTUNE_MIN_IQ_TRACKING_RATIO        (0.35f)  /* 常规对象总 Iq 跟踪比例下限。 */
/* 即使使用电流上限下的低响应回退，实测 Iq 仍必须保持足够强的相干性。 */
#define SPEED_AUTOTUNE_CEILING_MIN_IQ_TRACKING_RATIO (0.25f) /* 电流上限回退时的跟踪比例下限。 */
#define SPEED_AUTOTUNE_MAX_IQ_TRACKING_RATIO        (1.50f)  /* 实测/指令 Iq 跟踪比例上限。 */
#define SPEED_AUTOTUNE_MIN_IQ_COHERENCE             (0.60f)  /* 注入音调解释的 Iq 方差下限。 */
#define SPEED_AUTOTUNE_MIN_SPEED_COHERENCE          (0.35f)  /* 注入音调解释的速度方差下限。 */
#define SPEED_AUTOTUNE_MAX_CLIPPED_FRACTION         (0.05f)  /* 允许的电流控制器削波样本上限。 */
#define SPEED_AUTOTUNE_MIN_RESPONSE_RPM             (3.0f)   /* 预留的常规最小速度响应门限，rpm。 */

/*
 * 速度环目标自然频率由已辨识的电流环 Wi 推导。速度环至少比电流环慢 5 倍，并限制
 * 在同时适用于高、低机动性对象的实际范围内，避免级联环路带宽过于接近。
 */
#define SPEED_AUTOTUNE_CURRENT_TO_SPEED_RATIO       (5.0f)   /* 电流环/速度环目标带宽最小倍率。 */
#define SPEED_AUTOTUNE_MIN_NATURAL_FREQ_HZ          (0.50f)  /* 速度环目标自然频率下限，Hz。 */
#define SPEED_AUTOTUNE_MAX_NATURAL_FREQ_HZ          (2.00f)  /* 速度环目标自然频率上限，Hz。 */
#define SPEED_AUTOTUNE_TARGET_DAMPING_RATIO         (0.90f)  /* 二阶极点配置采用的目标阻尼比。 */

/*
 * 有界的低速连续运动验证。
 *
 * 首次施加目标以及正反向切换均使用三次 SmoothStep 平滑过渡。相同 Kp 必须连续通过
 * 正、反两个窗口的检查后才参与选择，避免单个有利齿槽区间产生不现实的小 Kp。
 * 低机动性对象在验证期间按比例加入少量模型 Ki，以克服仅用 P 时的静摩擦粘滑。
 */
#define SPEED_AUTOTUNE_KP_PROBE_TARGET_RPM          (8.0f)   /* 高机动性端点的 Kp 验证转速，rpm。 */
#define SPEED_AUTOTUNE_KP_PROBE_RAMP_MS             (500U)   /* 高机动性端点的目标斜坡时间，ms。 */
#define SPEED_AUTOTUNE_KP_PROBE_SETTLE_MS           (500U)   /* 高机动性端点的窗口稳定时间，ms。 */
#define SPEED_AUTOTUNE_KP_PROBE_MEASURE_MS          (500U)   /* 高机动性端点的窗口测量时间，ms。 */
/*
 * 低机动性对象不能在粘滑区内可靠评价，因此应在脱离静摩擦后的较高速度验证，并且
 * 只使用刻意削弱的积分项，避免积分储能后产生过大的起步电流。
 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_KP_TARGET_RPM   (12.0f)  /* 低机动性端点的 Kp 验证转速，rpm。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_KP_RAMP_MS      (2500U)  /* 低机动性端点的目标斜坡时间，ms。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_KP_SETTLE_MS    (1200U)  /* 低机动性端点的窗口稳定时间，ms。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_KP_MEASURE_MS   (1000U)  /* 低机动性端点的窗口测量时间，ms。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_INTEGRAL_RPM    (4.0f)   /* score=1 时允许积分的最低定向速度，rpm。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_KI_SCALE        (0.50f)  /* score=1 时探测/最终 Ki 的保留比例。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_I_LIMIT_RATIO   (0.80f)  /* 探测积分限幅占测试电流的比例。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_SEED_I_RATIO    (0.80f)  /* 摩擦种子 Kp 使用的正弦电流比例。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_MIN_SPEED_RATIO (-0.15f) /* 宽松窗口允许的最低速度/目标比例。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_KP_STEP_RATIO   (1.25f)  /* 低机动性端点的 Kp 递增倍率。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_MIN_MEAN_RATIO  (0.75f)  /* 宽松窗口平均速度比例下限。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_MAX_MEAN_RATIO  (1.40f)  /* 宽松窗口平均速度比例上限。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_MAX_SPEED_RATIO (2.50f)  /* 宽松窗口瞬时速度比例上限。 */
#define SPEED_AUTOTUNE_LOW_MOBILITY_MAX_RMS_RATIO   (0.70f)  /* 宽松窗口 RMS 误差比例上限。 */
#define SPEED_AUTOTUNE_KP_PROBE_REQUIRED_PASSES     (2U)     /* 每个 Kp 必须完成的方向窗口数量。 */
#define SPEED_AUTOTUNE_KP_PROBE_MIN_MEAN_RATIO      (0.85f)  /* 严格窗口平均速度比例下限。 */
#define SPEED_AUTOTUNE_KP_PROBE_MAX_MEAN_RATIO      (1.15f)  /* 严格窗口平均速度比例上限。 */
#define SPEED_AUTOTUNE_KP_PROBE_MIN_SPEED_RATIO     (0.25f)  /* 严格窗口瞬时速度比例下限。 */
#define SPEED_AUTOTUNE_KP_PROBE_MAX_SPEED_RATIO     (1.75f)  /* 严格窗口瞬时速度比例上限。 */
#define SPEED_AUTOTUNE_KP_PROBE_MAX_RMS_ERROR_RATIO (0.35f)  /* 严格窗口 RMS 误差比例上限。 */
#define SPEED_AUTOTUNE_KP_PROBE_FALLBACK_RMS_RATIO  (0.50f)  /* 高机动端宽松 RMS 误差比例上限。 */
#define SPEED_AUTOTUNE_KP_PROBE_STEP_RATIO          (1.40f)  /* 高机动性端点的 Kp 递增倍率。 */
#define SPEED_AUTOTUNE_KP_PROBE_MAX_ATTEMPTS        (16U)    /* 最多允许验证的不同 Kp 数量。 */
#define SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS         (32U)    /* 诊断历史最多保存的方向窗口数。 */
#define SPEED_AUTOTUNE_KP_PROBE_STOP_AFTER_WORSE    (2U)     /* 连续不改善多少个候选后停止搜索。 */
#define SPEED_AUTOTUNE_KP_PROBE_FULL_CURRENT_ERROR_RPM (2.00f) /* 推导 Kp 上限采用的速度误差，rpm。 */
#define SPEED_AUTOTUNE_KP_PROBE_STOP_RPM            (0.50f)  /* 低机动性端点停车窗口半宽，rpm。 */
#define SPEED_AUTOTUNE_KP_PROBE_STOP_RESET_RPM      (1.50f)  /* 低机动端停车计时复位转速，rpm。 */
#define SPEED_AUTOTUNE_KP_PROBE_BRAKE_TIMEOUT_MS    (4000U)  /* 低机动性端点制动超时，ms。 */
#define SPEED_AUTOTUNE_HIGH_MOBILITY_STOP_RPM       (1.50f)  /* 高机动性端点停车窗口半宽，rpm。 */
#define SPEED_AUTOTUNE_HIGH_MOBILITY_STOP_RESET_RPM (2.50f)  /* 高机动端停车计时复位转速，rpm。 */
#define SPEED_AUTOTUNE_HIGH_MOBILITY_BRAKE_TIMEOUT_MS (6000U) /* 高机动性端点制动超时，ms。 */

typedef enum
{
    SPEED_AUTOTUNE_IDLE = 0,       /* 参数已准备，首次 Handle 调用将复位 FOC。 */
    SPEED_AUTOTUNE_SETTLE,         /* 零 Iq 静置，等待电流和机械状态稳定。 */
    SPEED_AUTOTUNE_SINE_ADAPT,     /* 搜索能够产生有效速度响应的最小正弦电流。 */
    SPEED_AUTOTUNE_SINE_SETTLE,    /* 平滑淡入三音激励并丢弃过渡响应。 */
    SPEED_AUTOTUNE_SINE_MEASURE,   /* 累积各音调的同步解调和质量统计量。 */
    SPEED_AUTOTUNE_SINE_BRAKE,     /* 平滑淡出激励并主动阻尼至停止。 */
    SPEED_AUTOTUNE_SOLVE,          /* 检查 FRF 质量并计算模型 PI。 */
    SPEED_AUTOTUNE_KP_PROBE_RECOVER, /* 闭环 Kp 验证前重新预装载 FOC。 */
    SPEED_AUTOTUNE_KP_RAMP,        /* 平滑过渡到下一个带方向的速度目标。 */
    SPEED_AUTOTUNE_KP_PROBE,       /* 统计并评价一个恒速验证窗口。 */
    SPEED_AUTOTUNE_KP_PROBE_BRAKE, /* Kp 选定后将转子制动至稳定停止。 */
    SPEED_AUTOTUNE_FINISH,         /* 发布并写入已经验证的 PI 参数。 */
    SPEED_AUTOTUNE_RECOVER,        /* 重新预装载正常 FOC 并返回 RUN。 */
    SPEED_AUTOTUNE_FAULT           /* 故障锁定，PWM 关闭且结果无效。 */
} SpeedAutoTuneState_t;

typedef enum
{
    SPEED_AUTOTUNE_ERROR_NONE = 0,          /* 无错误。 */
    SPEED_AUTOTUNE_ERROR_OVERCURRENT,       /* 任一相电流超过独立安全上限。 */
    SPEED_AUTOTUNE_ERROR_OVERSPEED,         /* 独立窗口测速超过机械转速上限。 */
    SPEED_AUTOTUNE_ERROR_TRAVEL_LIMIT,      /* 累计机械位移超过允许圈数。 */
    SPEED_AUTOTUNE_ERROR_STAGE_TIMEOUT,     /* 制动或其他阶段未按时完成。 */
    SPEED_AUTOTUNE_ERROR_EXCITATION_TOO_SMALL, /* 电流上限下仍无足够速度响应。 */
    SPEED_AUTOTUNE_ERROR_CURRENT_TRACKING,  /* Iq 跟踪、相干性或削波率不合格。 */
    SPEED_AUTOTUNE_ERROR_INVALID_RESPONSE,  /* 速度 FRF 响应不足或有效音调过少。 */
    SPEED_AUTOTUNE_ERROR_INVALID_GAIN,      /* 模型或计算出的 PI 参数不合理。 */
    SPEED_AUTOTUNE_ERROR_FOC_DURATION,      /* 电流环执行时间超限。 */
    SPEED_AUTOTUNE_ERROR_KP_PROBE           /* 没有找到通过安全窗口的 Kp。 */
} SpeedAutoTuneError_t;

typedef struct
{
    /*
     * valid 是下列结果的统一提交标志。valid=false 时仍可能包含用于诊断的中间模型
     * 数据，但调用方必须等 Kp 验证通过且转子停止、valid=true 后才能使用最终参数。
     */
    bool valid;                    /* 最终结果是否已经完整验证并可供外部使用。 */
    SpeedAutoTuneState_t state;    /* 当前辨识状态机状态。 */
    SpeedAutoTuneError_t error;    /* 最近一次失败原因，成功或未运行时为 NONE。 */

    /*
     * 三音实验和同步解调诊断量。这里的幅值都是相干音调幅值，不是时域峰峰值。
     * coherence 表示注入音调能够解释的信号方差比例；clippedFraction 表示电流控制器
     * 发生输出削波的样本比例。
     */
    uint32_t sampleCount;          /* 三音测量阶段累计的速度速率样本数。 */
    uint32_t measuredCycles;       /* 三音测量覆盖的 0.5 Hz 基波完整周期数。 */
    float excitationCurrentA;      /* 自适应搜索选定的单正弦峰值电流，单位 A。 */
    float excitationFrequencyHz;  /* 基波激励频率，单位 Hz。 */
    float commandedIqAmplitudeA;  /* 三个指令音调合成后的相干总幅值，单位 A。 */
    float measuredIqAmplitudeA;   /* 三个实测 Iq 音调合成后的相干总幅值，单位 A。 */
    float speedAmplitudeRpm;       /* 三个速度音调合成后的机械转速幅值，单位 rpm。 */
    float currentTrackingRatio;   /* 实测 Iq 相干总幅值与指令总幅值之比。 */
    float iqCoherence;            /* 三个注入音调解释的 Iq 方差比例，[0,1]。 */
    float speedCoherence;         /* 三个注入音调解释的速度方差比例，[0,1]。 */
    float responsePhaseDeg;       /* 基波速度相对实测 Iq 的相位，单位度。 */
    float clippedFraction;        /* 电流控制器发生削波的样本比例，[0,1]。 */
    float excitationMobilityRpmPerA; /* 基波机械速度幅值/Iq 幅值，单位 rpm/A。 */
    float lowMobilityScore;       /* 低机动性权重：0=高机动，1=低机动。 */
    uint32_t validFrfToneCount;   /* 通过幅值、跟踪和相位门限的音调数量。 */
    float frfFrequencyHz[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 各音调频率，Hz。 */
    float frfMeasuredIqAmplitudeA[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 各音调实测 Iq 幅值，A。 */
    float frfSpeedAmplitudeRpm[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 各音调机械速度幅值，rpm。 */
    float frfPhaseDeg[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 各音调速度/Iq 相位，度。 */
    float frfAccelerationGain[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 各音调估算的加速度增益。 */

    /*
     * 局部对象模型：d(电气频率 Hz)/dt = accelerationGain * Iq_A。
     * modelKp/modelKi 由二阶极点配置得到，在后续低速验证成功前仍属于候选参数。
     */
    float accelerationGain_eHz_per_s_per_A; /* Iq 到电气加速度的辨识增益。 */
    float targetNaturalFrequencyHz; /* 受电流环带宽约束的目标自然频率，Hz。 */
    float targetDampingRatio;       /* 极点配置采用的目标阻尼比。 */
    float modelKp_A_per_eHz;        /* 纯模型计算的速度环 Kp，A/eHz。 */
    float modelKi_A_per_eHz_s;      /* 纯模型计算的速度环 Ki，A/(eHz*s)。 */

    /*
     * 双向低速验证和最终提交参数。kpSelectedByFallback=true 表示选中候选通过了较宽的
     * 安全窗口，但没有在正、反两个方向都通过严格跟踪窗口。
     */
    float probedKp_A_per_eHz;       /* 低速实测验证最终选中的 Kp，A/eHz。 */
    float kpProbeMeanRpm;           /* 选中候选最差方向窗口的平均定向速度，rpm。 */
    float kpProbeMinimumRpm;        /* 选中候选最差方向窗口的最小定向速度，rpm。 */
    float kpProbeMaximumRpm;        /* 选中候选最差方向窗口的最大定向速度，rpm。 */
    float kpProbeRmsErrorRpm;       /* 选中候选最差方向窗口的速度 RMS 误差，rpm。 */
    bool kpSelectedByFallback;      /* 是否由宽松安全窗口而非严格窗口选中。 */
    float kp_A_per_eHz;             /* 最终提交并写入速度控制器的 Kp，A/eHz。 */
    float ki_A_per_eHz_s;           /* 最终提交并写入速度控制器的 Ki，A/(eHz*s)。 */

    float testCurrentA;             /* 调用方授权的整个辨识过程电流上限，A。 */
} SpeedAutoTuneResult_t;

extern SpeedAutoTuneResult_t g_speedAutoTuneResult;

/**
 * 准备一次新的速度辨识；参数 <=0 A 时使用默认电流上限。
 * 本函数会清除上一次结果，但不会自行开启 PWM 或启动电机运动。
 */
void SpeedAutoTune_Start(float testCurrentA);

/** 中止辨识、清除控制状态并关闭 PWM。 */
void SpeedAutoTune_Abort(void);

/**
 * 执行一次 PWM 频率的电流控制和速度辨识处理。机械量估计及状态机更新会在函数内部
 * 分频到 SPEED_CONTROL_RATE 执行。
 */
void SpeedAutoTune_Handle(void);

/** 返回最近一次平滑速度辨识的结果结构体，只读使用。 */
const SpeedAutoTuneResult_t *SpeedAutoTune_GetResult(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTORCONTROL_CONTROL_SPEED_AUTOTUNE_H_：速度辨识头文件保护结束。 */
