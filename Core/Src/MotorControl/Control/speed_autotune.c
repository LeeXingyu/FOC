/*
 * speed_autotune.c
 *
 * 速度环辨识总体流程：
 *
 * 1. 使用平滑的 0.5 Hz 单正弦逐级搜索测试电流，使速度响应足够测量，同时不超过
 *    调用方授权的电流上限。
 * 2. 使用相干的 0.5/1.0/1.5 Hz 三音电流同时激励机械对象。每个音调分别做同步
 *    解调，以抑制编码器量化、齿槽谐波、直流偏置和电流换向延迟等非相干成分。
 * 3. 由各音调的实测 Iq 和电气速度复数幅值计算局部加速度增益。组合模型时使用
 *    (Iq/omega)^2 加权，防止响应微弱、信噪比较低的高频点主导结果。
 *
 * 对局部低速对象采用模型：
 *
 *     d(felectrical)/dt = b * Iq
 *
 * 其正弦稳态速度/电流幅值比为 |G(jw)| = b/omega，因此：
 *
 *     b = omega * speedAmplitude / currentAmplitude
 *
 * 4. 根据目标自然频率和阻尼比做极点配置，得到模型 Kp/Ki。
 * 5. 通过平滑斜坡的正、反向低速实验逐级验证 Kp。连续的低机动性权重决定验证时
 *    加入多少 Ki，也决定候选选择更偏向低 RMS 误差还是较高比例刚度。
 * 6. 最终 Ki 随已验证 Kp 同比例缩放，以保持模型 PI 零点位置，然后停车、提交结果
 *    并恢复正常 FOC。
 *
 * 状态流如下，任何故障出口都会立即关闭 PWM：
 *
 *   IDLE -> SETTLE -> SINE_ADAPT -> SINE_SETTLE -> SINE_MEASURE
 *        -> SINE_BRAKE -> SOLVE -> KP_PROBE_RECOVER
 *        -> {KP_RAMP <-> KP_PROBE} -> KP_PROBE_BRAKE
 *        -> FINISH -> RECOVER -> 正常 RUN
 *
 * `SpeedAutoTune_Handle()` 按 PWM/电流环频率调用；编码器运动估计、同步解调采样和
 * 状态计时每 SPEED_CONTROL_COUNT 次 PWM 调用才更新一次，即工作在
 * SPEED_CONTROL_RATE。阅读所有 tick 计数器时必须区分这两个时间基准。
 */

#include "speed_autotune.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "curr_fbdk.h"
#include "main.h"
#include "mc_interface.h"
#include "motor_control.h"
#include "speed_pos_fbdk.h"
#include "stm32g4xx_ll_tim.h"

#define SPEED_AUTOTUNE_PI               (3.14159265358979323846f)
#define SPEED_AUTOTUNE_RAD_TO_DEG       (57.295779513082320876f)
#define SPEED_AUTOTUNE_MS_TO_TICKS(ms_) \
    (((uint32_t)(ms_) * (uint32_t)SPEED_CONTROL_RATE) / 1000U)
#define SPEED_AUTOTUNE_GUARD_SAMPLES \
    SPEED_AUTOTUNE_MS_TO_TICKS(SPEED_AUTOTUNE_GUARD_WINDOW_MS)

static const float s_multisineFrequencyHz[
    SPEED_AUTOTUNE_MULTISINE_TONE_COUNT] =
{
    0.50f, 1.00f, 1.50f
};

/* 1、2、3 次谐波采用的低峰值因数初相位，减小三音叠加后的瞬时电流峰值。 */
static const float s_multisinePhaseRad[
    SPEED_AUTOTUNE_MULTISINE_TONE_COUNT] =
{
    0.0f, 0.5f * SPEED_AUTOTUNE_PI, 0.0f
};

typedef struct
{
    bool valid;            /* 该状态是否曾进入，决定历史记录能否打印。 */
    float mechanicalRpm;   /* 进入该状态瞬间的独立窗口机械转速，rpm。 */
    float iqRefA;          /* 进入该状态前一拍的 q 轴参考电流，A。 */
    float iqMeasA;         /* 进入该状态前一拍的 q 轴实测电流，A。 */
    uint32_t progress;     /* 当时的正弦自适应周期数或 Kp 探测次数。 */
} SpeedStageSnapshot_t;

typedef struct
{
    /* PWM 频率到速度控制频率的分频计数，以及按相应时间基准记录的计时器。 */
    uint32_t speedDivider;     /* PWM 调用分频计数，达到 SPEED_CONTROL_COUNT 后清零。 */
    uint32_t stageTicks;       /* 当前状态已运行的速度控制周期数。 */
    uint32_t totalTicks;       /* 本次辨识累计的速度控制周期数。 */
    uint32_t stopStableTicks;  /* 转速连续处于停车窗口内的速度控制周期数。 */
    uint32_t recoveryHfTicks;  /* PWM 关闭后等待的高频恢复调用次数。 */

    /*
     * 独立的编码器展开路径，用于位移/超速保护和低于常规测速死区的辨识。这里有意
     * 不复用正常速度环的卡尔曼速度，避免低速死区把真实微小运动置零。
     */
    uint32_t previousRaw;          /* 上一次编码器单圈原始位置。 */
    int64_t unwrappedCounts;       /* 从实验起点累计展开的有符号编码器计数。 */
    int64_t guardWindowStartCounts; /* 当前保护测速窗口起点的展开计数。 */
    uint32_t guardWindowSamples;   /* 当前保护测速窗口已累计的样本数。 */
    float guardMechanicalRpm;      /* 最近保护窗口估算的机械转速，rpm。 */
    float guardIqSumA;             /* 当前保护窗口实测 Iq 的累加值，A。 */
    float guardMeanIqA;            /* 最近完整保护窗口的平均实测 Iq，A。 */
    bool guardValid;               /* 是否已经形成至少一个完整保护测速窗口。 */

    /* 自适应正弦指令、回中阻尼及单周期相干速度响应。 */
    float sineCurrentA;            /* 当前单正弦峰值电流，也是后续三音幅值基准，A。 */
    float sineCommandA;            /* 当前 PWM 周期使用的合成 q 轴电流指令，A。 */
    float sineCenterKp;            /* 抑制净位移的回中速度 P 阻尼，A/eHz。 */
    int8_t sineBrakeDirection;     /* 正弦制动阶段最近一次非零转速方向，-1/0/+1。 */
    int8_t kpBrakeDirection;       /* Kp 制动阶段最近一次非零制动电流方向。 */
    float sineAdaptSpeedSinSum;    /* 自适应周期速度对基波 sin 的投影累加。 */
    float sineAdaptSpeedCosSum;    /* 自适应周期速度对基波 cos 的投影累加。 */
    uint32_t sineAdaptSamples;     /* 当前自适应周期参与同步解调的样本数。 */
    uint32_t sineAdaptDwellCycles; /* 当前电流幅值已经停留的完整周期数。 */
    uint32_t sineAdaptCycles;      /* 已评价的有效自适应周期总数。 */
    bool sineCeilingResponseAccepted; /* 是否在电流上限采用低响应回退规则。 */

    /*
     * 每个注入音调对应的指令 Iq、实测 Iq 和电气速度同相/正交累加器。另行保留直流
     * 与总能量统计，用于计算相干性和削波质量门限。
     */
    float iqSinSum[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 实测 Iq 的 sin 投影和。 */
    float iqCosSum[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 实测 Iq 的 cos 投影和。 */
    float commandSinSum[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 指令 Iq 的 sin 投影和。 */
    float commandCosSum[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 指令 Iq 的 cos 投影和。 */
    float speedSinSum[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 电气速度的 sin 投影和。 */
    float speedCosSum[SPEED_AUTOTUNE_MULTISINE_TONE_COUNT]; /* 电气速度的 cos 投影和。 */
    float iqSum;                    /* 全部测量样本的实测 Iq 累加，用于求均值。 */
    float speedSum;                 /* 全部测量样本的电气速度累加，用于求均值。 */
    float iqSquareSum;              /* 实测 Iq 平方和，用于求去直流总方差。 */
    float speedSquareSum;           /* 电气速度平方和，用于求去直流总方差。 */
    uint32_t sineSamples;           /* 三音测量阶段累计的有效样本数。 */
    uint32_t sineClippedSamples;    /* 电流控制器报告削波的样本数。 */

    /* 当前 Kp 候选和单个带方向速度窗口的统计量。 */
    float kpProbeGain;              /* 当前正在验证的速度环 Kp，A/eHz。 */
    float kpProbeMaximumGain;       /* 由测试电流上限推导的 Kp 搜索上限。 */
    float kpProbeIntegralA;         /* 低机动性验证专用的受限积分电流，A。 */
    float kpBrakeStartRpm;          /* 接受 Kp 后开始制动时的机械转速，rpm。 */
    float kpProbeTargetRpm;         /* 当前验证方向的带符号目标转速，rpm。 */
    float kpRampStartRpm;           /* 当前 SmoothStep 斜坡起点转速，rpm。 */
    float kpRampEndRpm;             /* 当前 SmoothStep 斜坡终点转速，rpm。 */
    float kpProbeSpeedSumRpm;       /* 当前测量窗口的定向速度累加值。 */
    float kpProbeErrorSquareSum;    /* 当前测量窗口的速度误差平方和。 */
    float kpProbeMinimumRpm;        /* 当前测量窗口的最小定向速度，rpm。 */
    float kpProbeMaximumRpm;        /* 当前测量窗口的最大定向速度，rpm。 */
    uint32_t kpProbeSamples;        /* 当前测量窗口累计的样本数。 */
    uint32_t kpProbeAttempts;       /* 已尝试的不同 Kp 数量，初始候选计为一次。 */
    uint32_t kpProbePasses;         /* 当前候选通过严格窗口的方向数量。 */

    /* 汇总同一个 Kp 对应的正向和反向窗口结果。 */
    uint32_t kpCandidateWindows;    /* 当前 Kp 已完成的方向窗口数量。 */
    bool kpCandidateStrict;         /* 所有已测方向是否都通过严格跟踪门限。 */
    bool kpCandidateSafe;           /* 所有已测方向是否都通过宽松安全门限。 */
    float kpCandidateWorstRmsRpm;   /* 两个方向中较大的 RMS 速度误差，rpm。 */
    float kpCandidateMeanRpm;       /* 最差 RMS 窗口的平均定向速度，rpm。 */
    float kpCandidateMinimumRpm;    /* 最差 RMS 窗口的最小定向速度，rpm。 */
    float kpCandidateMaximumRpm;    /* 最差 RMS 窗口的最大定向速度，rpm。 */

    /* 按机动性相关效用函数选出的最佳安全候选。 */
    bool kpBestValid;               /* 是否已经存在通过宽松安全门限的候选。 */
    bool kpBestStrict;              /* 当前最佳候选是否同时通过两个严格窗口。 */
    float kpBestGain;               /* 当前最佳候选 Kp，A/eHz。 */
    float kpBestWorstRmsRpm;        /* 当前最佳候选的最差 RMS 误差，rpm。 */
    float kpBestMeanRpm;            /* 当前最佳候选最差窗口的平均速度，rpm。 */
    float kpBestMinimumRpm;         /* 当前最佳候选最差窗口的最小速度，rpm。 */
    float kpBestMaximumRpm;         /* 当前最佳候选最差窗口的最大速度，rpm。 */
    float kpBestUtility;            /* 当前最佳候选的机动性加权效用值。 */
    uint32_t kpWorseCandidates;     /* 最佳值之后连续未改善的候选数量。 */

    /* 每个 Kp/方向窗口的诊断历史，探测失败时逐项打印。 */
    float kpWindowGain[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口使用的 Kp。 */
    float kpWindowTargetRpm[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口带符号目标转速。 */
    float kpWindowMeanRpm[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口平均定向速度。 */
    float kpWindowMinimumRpm[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口最小定向速度。 */
    float kpWindowMaximumRpm[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口最大定向速度。 */
    float kpWindowRmsErrorRpm[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口 RMS 误差。 */
    bool kpWindowPassed[SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS]; /* 各窗口是否通过严格门限。 */
    uint32_t kpWindowCount;         /* 已写入诊断历史的窗口数量。 */

    SpeedStageSnapshot_t stageSnapshots[SPEED_AUTOTUNE_FAULT + 1U]; /* 各状态首次/最近进入快照。 */
    bool stageHistoryPrinted;      /* 状态历史是否已经打印，防止重复输出。 */

    /*
     * 辨识前速度 PI 快照。目前只保存而未被读取，现有中止/故障路径不会自动恢复它们。
     */
    float oldKp;                   /* 辨识开始前的速度环 Kp，A/eHz。 */
    float oldKi;                   /* 辨识开始前的速度环 Ki，A/(eHz*s)。 */
} SpeedAutoTuneRuntime_t;

SpeedAutoTuneResult_t g_speedAutoTuneResult;
static SpeedAutoTuneRuntime_t s_speedAutoTune;

static float SpeedAutoTune_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static float SpeedAutoTune_BlendProfile(float highMobilityValue,
                                         float lowMobilityValue)
{
    /*
     * lowMobilityScore 是机动性的反向权重：
     *   score=0 -> 返回高机动性端点
     *   score=1 -> 返回低机动性端点
     * 分数生成时已经限制到 [0,1]，调用方可直接线性插值，无需重复做边界检查。
     */
    return highMobilityValue +
           (lowMobilityValue - highMobilityValue) *
           g_speedAutoTuneResult.lowMobilityScore;
}

static const char *SpeedAutoTune_GetPlantProfileName(void)
{
    if (g_speedAutoTuneResult.lowMobilityScore <= 0.25f)
    {
        return "high-mobility";
    }
    if (g_speedAutoTuneResult.lowMobilityScore >= 0.75f)
    {
        return "low-mobility";
    }
    return "mixed-mobility";
}

static const char *SpeedAutoTune_GetSelectionName(void)
{
    if (g_speedAutoTuneResult.lowMobilityScore <= 0.25f)
    {
        return "best-rms";
    }
    if (g_speedAutoTuneResult.lowMobilityScore >= 0.75f)
    {
        return "highest-safe";
    }
    return "blended";
}

static void SpeedAutoTune_UpdateMobilityProfile(float speedAmplitudeRpm)
{
    float currentA = fmaxf(s_speedAutoTune.sineCurrentA, 1.0e-6f);
    float mobilityRpmPerA = speedAmplitudeRpm / currentA;
    float mobilitySpan = SPEED_AUTOTUNE_MOBILITY_HIGH_RPM_PER_A -
                         SPEED_AUTOTUNE_MOBILITY_LOW_RPM_PER_A;
    /*
     * 将机械速度基波响应转换为归一化的反向机动性。rpm/A 越大，分数越接近 0；
     * rpm/A 越小，分数越接近 1。因此后续乘以该分数的补偿只会随对象变得更难驱动
     * 而增强。
     */
    float lowMobilityScore =
        (SPEED_AUTOTUNE_MOBILITY_HIGH_RPM_PER_A - mobilityRpmPerA) /
        mobilitySpan;

    g_speedAutoTuneResult.excitationMobilityRpmPerA = mobilityRpmPerA;
    g_speedAutoTuneResult.lowMobilityScore = SpeedAutoTune_Clamp(
        lowMobilityScore, 0.0f, 1.0f);

    printf("[SPEED_AUTOTUNE] plant profile: mobility=%.3f rpm/A, "
           "lowMobilityScore=%.3f, profile=%s\n",
           mobilityRpmPerA,
           g_speedAutoTuneResult.lowMobilityScore,
           SpeedAutoTune_GetPlantProfileName());
}

static float SpeedAutoTune_SmoothStep(float value)
{
    float x = SpeedAutoTune_Clamp(value, 0.0f, 1.0f);

    return x * x * (3.0f - 2.0f * x);
}

static uint32_t SpeedAutoTune_GetSinePeriodTicks(void)
{
    float ticks = (float)SPEED_CONTROL_RATE /
                  SPEED_AUTOTUNE_SINE_FREQUENCY_HZ;

    return (uint32_t)(ticks + 0.5f);
}

static float SpeedAutoTune_GetSineAtTick(uint32_t tick, float amplitudeA)
{
    float phase = 2.0f * SPEED_AUTOTUNE_PI *
                  SPEED_AUTOTUNE_SINE_FREQUENCY_HZ *
                  ((float)tick / (float)SPEED_CONTROL_RATE);

    return amplitudeA * sinf(phase);
}

static float SpeedAutoTune_GetMultisineAtTick(uint32_t tick,
                                               float amplitudeA)
{
    float timeS = (float)tick / (float)SPEED_CONTROL_RATE;
    float toneAmplitudeA = amplitudeA *
        SPEED_AUTOTUNE_MULTISINE_PER_TONE_RATIO;
    float commandA = 0.0f;

    for (uint32_t tone = 0U;
         tone < SPEED_AUTOTUNE_MULTISINE_TONE_COUNT;
         tone++)
    {
        float phase = 2.0f * SPEED_AUTOTUNE_PI *
                      s_multisineFrequencyHz[tone] * timeS +
                      s_multisinePhaseRad[tone];
        commandA += toneAmplitudeA * sinf(phase);
    }

    return commandA;
}

static void SpeedAutoTune_ResetSineAdaptCycle(void)
{
    s_speedAutoTune.sineAdaptSpeedSinSum = 0.0f;
    s_speedAutoTune.sineAdaptSpeedCosSum = 0.0f;
    s_speedAutoTune.sineAdaptSamples = 0U;
}

static void SpeedAutoTune_ResetSineMeasurement(void)
{
    memset(s_speedAutoTune.iqSinSum, 0,
           sizeof(s_speedAutoTune.iqSinSum));
    memset(s_speedAutoTune.iqCosSum, 0,
           sizeof(s_speedAutoTune.iqCosSum));
    memset(s_speedAutoTune.commandSinSum, 0,
           sizeof(s_speedAutoTune.commandSinSum));
    memset(s_speedAutoTune.commandCosSum, 0,
           sizeof(s_speedAutoTune.commandCosSum));
    memset(s_speedAutoTune.speedSinSum, 0,
           sizeof(s_speedAutoTune.speedSinSum));
    memset(s_speedAutoTune.speedCosSum, 0,
           sizeof(s_speedAutoTune.speedCosSum));
    s_speedAutoTune.iqSum = 0.0f;
    s_speedAutoTune.speedSum = 0.0f;
    s_speedAutoTune.iqSquareSum = 0.0f;
    s_speedAutoTune.speedSquareSum = 0.0f;
    s_speedAutoTune.sineSamples = 0U;
    s_speedAutoTune.sineClippedSamples = 0U;
}

static void SpeedAutoTune_ResetKpProbeMeasurement(void)
{
    s_speedAutoTune.kpProbeSpeedSumRpm = 0.0f;
    s_speedAutoTune.kpProbeErrorSquareSum = 0.0f;
    s_speedAutoTune.kpProbeMinimumRpm = INFINITY;
    s_speedAutoTune.kpProbeMaximumRpm = -INFINITY;
    s_speedAutoTune.kpProbeSamples = 0U;
}

static float SpeedAutoTune_GetKpValidationTargetRpm(void)
{
    return SpeedAutoTune_BlendProfile(
        SPEED_AUTOTUNE_KP_PROBE_TARGET_RPM,
        SPEED_AUTOTUNE_LOW_MOBILITY_KP_TARGET_RPM);
}

static uint32_t SpeedAutoTune_GetKpRampMs(void)
{
    return (uint32_t)(SpeedAutoTune_BlendProfile(
        (float)SPEED_AUTOTUNE_KP_PROBE_RAMP_MS,
        (float)SPEED_AUTOTUNE_LOW_MOBILITY_KP_RAMP_MS) + 0.5f);
}

static uint32_t SpeedAutoTune_GetKpSettleMs(void)
{
    return (uint32_t)(SpeedAutoTune_BlendProfile(
        (float)SPEED_AUTOTUNE_KP_PROBE_SETTLE_MS,
        (float)SPEED_AUTOTUNE_LOW_MOBILITY_KP_SETTLE_MS) + 0.5f);
}

static uint32_t SpeedAutoTune_GetKpMeasureMs(void)
{
    return (uint32_t)(SpeedAutoTune_BlendProfile(
        (float)SPEED_AUTOTUNE_KP_PROBE_MEASURE_MS,
        (float)SPEED_AUTOTUNE_LOW_MOBILITY_KP_MEASURE_MS) + 0.5f);
}

static float SpeedAutoTune_GetKpStepRatio(void)
{
    return SpeedAutoTune_BlendProfile(
        SPEED_AUTOTUNE_KP_PROBE_STEP_RATIO,
        SPEED_AUTOTUNE_LOW_MOBILITY_KP_STEP_RATIO);
}

static float SpeedAutoTune_GetKpBrakeStopRpm(void)
{
    return SpeedAutoTune_BlendProfile(
        SPEED_AUTOTUNE_HIGH_MOBILITY_STOP_RPM,
        SPEED_AUTOTUNE_KP_PROBE_STOP_RPM);
}

static float SpeedAutoTune_GetKpBrakeResetRpm(void)
{
    return SpeedAutoTune_BlendProfile(
        SPEED_AUTOTUNE_HIGH_MOBILITY_STOP_RESET_RPM,
        SPEED_AUTOTUNE_KP_PROBE_STOP_RESET_RPM);
}

static uint32_t SpeedAutoTune_GetKpBrakeTimeoutMs(void)
{
    return (uint32_t)(SpeedAutoTune_BlendProfile(
        (float)SPEED_AUTOTUNE_HIGH_MOBILITY_BRAKE_TIMEOUT_MS,
        (float)SPEED_AUTOTUNE_KP_PROBE_BRAKE_TIMEOUT_MS) + 0.5f);
}

static float SpeedAutoTune_GetKpFeedbackRpm(void)
{
    /*
     * Kp/PI 验证必须使用与正式速度环完全相同的卡尔曼滤波轴端速度。独立的 8 ms
     * 原始位置窗口仍用于死区以下的正弦辨识和安全保护，但它包含的齿槽/齿波纹不能
     * 在约 12 rpm 的运动验证点被误当成闭环速度误差。
     */
    return g_axis.fbdk.fSpeedKalman;
}

static float SpeedAutoTune_GetCenteredSineCommand(uint32_t tick,
                                                   float forcingScale)
{
    float targetElectricalHz =
        SPEED_AUTOTUNE_SINE_TARGET_AMPLITUDE_RPM *
        (float)POLE_PAIR_NUM / 60.0f;
    float targetCenterKp =
        SPEED_AUTOTUNE_SINE_CENTER_CURRENT_RATIO *
        s_speedAutoTune.sineCurrentA / targetElectricalHz;
    float rampTicks = (float)SPEED_AUTOTUNE_MS_TO_TICKS(
        SPEED_AUTOTUNE_SINE_CENTER_GAIN_RAMP_MS);
    /*
     * 正弦辨识使用本模块自己的原始位置窗口，而不是 fSpeedKalman。正常卡尔曼输入会
     * 有意把很小的 1 ms 编码器增量置零；对 MT6835 而言该死区约为 3.66 rpm，几乎
     * 覆盖整个 4 rpm 辨识目标。8 ms 窗口能保留真实的小幅运动，后续同步解调再抑制
     * 由此带来的噪声。
     */
    float speedElectricalHz = s_speedAutoTune.guardMechanicalRpm *
                              (float)POLE_PAIR_NUM / 60.0f;
    float forcingA;

    /*
     * 自适应正弦幅值变化时，对回中增益做渐变而不是阶跃。这样既消除近似无摩擦转子
     * 的中性速度漂移，又能让振荡保持在原点附近且不产生转矩突变。后续辨识使用实际
     * 合成后的总 Iq，因此回中阻尼不会被漏出模型输入。
     */
    if (rampTicks > 1.0f)
    {
        s_speedAutoTune.sineCenterKp +=
            (targetCenterKp - s_speedAutoTune.sineCenterKp) / rampTicks;
    }
    else
    {
        s_speedAutoTune.sineCenterKp = targetCenterKp;
    }

    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_SINE_SETTLE ||
        g_speedAutoTuneResult.state == SPEED_AUTOTUNE_SINE_MEASURE ||
        g_speedAutoTuneResult.state == SPEED_AUTOTUNE_SINE_BRAKE)
    {
        forcingA = forcingScale * SpeedAutoTune_GetMultisineAtTick(
            tick, s_speedAutoTune.sineCurrentA);
    }
    else
    {
        forcingA = forcingScale * SpeedAutoTune_GetSineAtTick(
            tick, s_speedAutoTune.sineCurrentA);
    }
    /*
     * Iq 指令由辨识激励和零速附近的弱 P 阻尼组成：
     *
     *   Iq_cmd = Iq_forcing - Kcenter * electrical_speed
     *
     * 阻尼项用于限制累计位移，同时它也包含在指令/实测同步解调中，因此不会被错误地
     * 当作未知对象响应的一部分。
     */
    return SpeedAutoTune_Clamp(
        forcingA - s_speedAutoTune.sineCenterKp * speedElectricalHz,
        -g_speedAutoTuneResult.testCurrentA,
        g_speedAutoTuneResult.testCurrentA);
}

static const char *SpeedAutoTune_StateName(SpeedAutoTuneState_t state)
{
    switch (state)
    {
        case SPEED_AUTOTUNE_IDLE: return "IDLE";
        case SPEED_AUTOTUNE_SETTLE: return "SETTLE";
        case SPEED_AUTOTUNE_SINE_ADAPT: return "SINE_ADAPT";
        case SPEED_AUTOTUNE_SINE_SETTLE: return "SINE_SETTLE";
        case SPEED_AUTOTUNE_SINE_MEASURE: return "SINE_MEASURE";
        case SPEED_AUTOTUNE_SINE_BRAKE: return "SINE_BRAKE";
        case SPEED_AUTOTUNE_SOLVE: return "SOLVE";
        case SPEED_AUTOTUNE_KP_PROBE_RECOVER: return "KP_PROBE_RECOVER";
        case SPEED_AUTOTUNE_KP_RAMP: return "KP_RAMP";
        case SPEED_AUTOTUNE_KP_PROBE: return "KP_PROBE";
        case SPEED_AUTOTUNE_KP_PROBE_BRAKE: return "KP_PROBE_BRAKE";
        case SPEED_AUTOTUNE_FINISH: return "FINISH";
        case SPEED_AUTOTUNE_RECOVER: return "RECOVER";
        case SPEED_AUTOTUNE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *SpeedAutoTune_ErrorName(SpeedAutoTuneError_t error)
{
    switch (error)
    {
        case SPEED_AUTOTUNE_ERROR_NONE: return "none";
        case SPEED_AUTOTUNE_ERROR_OVERCURRENT: return "overcurrent";
        case SPEED_AUTOTUNE_ERROR_OVERSPEED: return "overspeed";
        case SPEED_AUTOTUNE_ERROR_TRAVEL_LIMIT: return "travel limit";
        case SPEED_AUTOTUNE_ERROR_STAGE_TIMEOUT: return "stage timeout";
        case SPEED_AUTOTUNE_ERROR_EXCITATION_TOO_SMALL:
            return "sine excitation";
        case SPEED_AUTOTUNE_ERROR_CURRENT_TRACKING:
            return "current tracking";
        case SPEED_AUTOTUNE_ERROR_INVALID_RESPONSE:
            return "invalid multisine response";
        case SPEED_AUTOTUNE_ERROR_INVALID_GAIN: return "invalid gain";
        case SPEED_AUTOTUNE_ERROR_FOC_DURATION: return "FOC duration";
        case SPEED_AUTOTUNE_ERROR_KP_PROBE: return "low-speed Kp probe";
        default: return "unknown";
    }
}

static void SpeedAutoTune_EnterState(SpeedAutoTuneState_t state)
{
    SpeedStageSnapshot_t *snapshot;

    g_speedAutoTuneResult.state = state;
    s_speedAutoTune.stageTicks = 0U;
    s_speedAutoTune.stopStableTicks = 0U;

    /*
     * 在新状态改变指令之前记录入口条件。发生故障时可据此区分“对象响应不合格”和
     * “状态切换不干净”，例如进入下一阶段时仍有残余速度或电流。
     */
    snapshot = &s_speedAutoTune.stageSnapshots[state];
    snapshot->valid = true;
    snapshot->mechanicalRpm = s_speedAutoTune.guardMechanicalRpm;
    snapshot->iqRefA =
        FIXP30_toF(g_axis.currCtrl.refIdq.Q) * CURRENT_SCALE;
    snapshot->iqMeasA =
        FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE;
    snapshot->progress = (state <= SPEED_AUTOTUNE_SINE_BRAKE) ?
                         s_speedAutoTune.sineAdaptCycles :
                         s_speedAutoTune.kpProbeAttempts;
}

static void SpeedAutoTune_PrintStageHistory(void)
{
    if (s_speedAutoTune.stageHistoryPrinted)
    {
        return;
    }

    s_speedAutoTune.stageHistoryPrinted = true;
    for (uint32_t state = (uint32_t)SPEED_AUTOTUNE_SETTLE;
         state <= (uint32_t)SPEED_AUTOTUNE_SOLVE;
         state++)
    {
        const SpeedStageSnapshot_t *snapshot =
            &s_speedAutoTune.stageSnapshots[state];

        if (!snapshot->valid)
        {
            continue;
        }

        printf("[SPEED_AUTOTUNE] stage=%s, rpm=%.3f, prevIqRef=%.3f A, "
               "prevIqMeas=%.3f A, progress=%lu\n",
               SpeedAutoTune_StateName((SpeedAutoTuneState_t)state),
               snapshot->mechanicalRpm,
               snapshot->iqRefA,
               snapshot->iqMeasA,
               (unsigned long)snapshot->progress);
    }
}

static void SpeedAutoTune_Fail(SpeedAutoTuneError_t error)
{
    SpeedAutoTuneState_t failedState = g_speedAutoTuneResult.state;

    /*
     * 故障采用关闭优先策略：先关闭 PWM，再打印诊断；同时清除结果提交标志并把轴
     * 切到故障状态。任何中间模型或 Kp 候选都不能以有效结果形式被外部使用。
     */
    SwitchOff_PWM(g_axis.pPWMCHandle);
    SpeedAutoTune_PrintStageHistory();
    MC_Reset_Control_State();
    g_speedAutoTuneResult.valid = false;
    g_speedAutoTuneResult.error = error;
    g_speedAutoTuneResult.state = SPEED_AUTOTUNE_FAULT;
    g_bStartSpeedAutoTune = false;
    g_axis.uFaultOccurred |= MC_SW_ERROR;
    g_axis.state = AXIS_STATE_FAULT_NOW;

    printf("[SPEED_AUTOTUNE] failed: stage=%s, error=%d (%s), "
           "rpm=%.3f, sineIq=%.3f A, adaptAmp=%.3f rpm, measuredIq=%.3f A, "
           "travel=%.3f rev, samples=%lu\n",
           SpeedAutoTune_StateName(failedState),
           (int)error,
           SpeedAutoTune_ErrorName(error),
           s_speedAutoTune.guardMechanicalRpm,
           s_speedAutoTune.sineCurrentA,
           g_speedAutoTuneResult.speedAmplitudeRpm,
           s_speedAutoTune.guardMeanIqA,
           fabsf((float)s_speedAutoTune.unwrappedCounts /
                 (float)ENCODER_COUNT),
           (unsigned long)s_speedAutoTune.sineSamples);

    if (failedState == SPEED_AUTOTUNE_KP_RAMP ||
        failedState == SPEED_AUTOTUNE_KP_PROBE ||
        failedState == SPEED_AUTOTUNE_KP_PROBE_RECOVER ||
        failedState == SPEED_AUTOTUNE_KP_PROBE_BRAKE)
    {
        float minimumRpm = 0.0f;
        float maximumRpm = 0.0f;

        if (s_speedAutoTune.kpProbeSamples > 0U)
        {
            minimumRpm = s_speedAutoTune.kpProbeMinimumRpm;
            maximumRpm = s_speedAutoTune.kpProbeMaximumRpm;
        }

        printf("[SPEED_AUTOTUNE] Kp probe failed: attempts=%lu, "
               "passes=%lu/%u, lastKp=%.6f, maxKp=%.6f, "
               "target=%.2f rpm, mean=%.3f rpm, "
               "range=%.3f..%.3f rpm, samples=%lu, "
               "brakeStable=%lu/%lu ticks\n",
               (unsigned long)s_speedAutoTune.kpProbeAttempts,
               (unsigned long)s_speedAutoTune.kpProbePasses,
               (unsigned int)SPEED_AUTOTUNE_KP_PROBE_REQUIRED_PASSES,
               s_speedAutoTune.kpProbeGain,
               s_speedAutoTune.kpProbeMaximumGain,
               s_speedAutoTune.kpProbeTargetRpm,
               g_speedAutoTuneResult.kpProbeMeanRpm,
               minimumRpm,
               maximumRpm,
               (unsigned long)s_speedAutoTune.kpProbeSamples,
               (unsigned long)s_speedAutoTune.stopStableTicks,
               (unsigned long)SPEED_AUTOTUNE_MS_TO_TICKS(
                   SPEED_AUTOTUNE_STOP_STABLE_MS));

        for (uint32_t index = 0U;
             index < s_speedAutoTune.kpWindowCount;
             index++)
        {
            printf("[SPEED_AUTOTUNE] Kp window[%lu]: target=%+.1f rpm, "
                   "Kp=%.6f, directedMean=%.3f rpm, "
                   "range=%.3f..%.3f rpm, rmsError=%.3f rpm, pass=%u\n",
                   (unsigned long)index,
                   s_speedAutoTune.kpWindowTargetRpm[index],
                   s_speedAutoTune.kpWindowGain[index],
                   s_speedAutoTune.kpWindowMeanRpm[index],
                   s_speedAutoTune.kpWindowMinimumRpm[index],
                   s_speedAutoTune.kpWindowMaximumRpm[index],
                   s_speedAutoTune.kpWindowRmsErrorRpm[index],
                   s_speedAutoTune.kpWindowPassed[index] ? 1U : 0U);
        }
    }
}

static float SpeedAutoTune_GetPhaseCurrentPeakA(void)
{
    float phaseR =
        fabsf(FIXP30_toF(g_axis.currCtrl.IrstMeas.R) * CURRENT_SCALE);
    float phaseS =
        fabsf(FIXP30_toF(g_axis.currCtrl.IrstMeas.S) * CURRENT_SCALE);
    float phaseT =
        fabsf(FIXP30_toF(g_axis.currCtrl.IrstMeas.T) * CURRENT_SCALE);
    float peak = phaseR;

    if (phaseS > peak)
    {
        peak = phaseS;
    }
    if (phaseT > peak)
    {
        peak = phaseT;
    }
    return peak;
}

static int32_t SpeedAutoTune_GetWrappedDelta(uint32_t raw,
                                             uint32_t previousRaw)
{
    int32_t delta = (int32_t)raw - (int32_t)previousRaw;
    int32_t halfCount = (int32_t)ENCODER_COUNT / 2;

    if (delta > halfCount)
    {
        delta -= (int32_t)ENCODER_COUNT;
    }
    else if (delta < -halfCount)
    {
        delta += (int32_t)ENCODER_COUNT;
    }
    return delta;
}

static void SpeedAutoTune_UpdateKinematics(void)
{
    uint32_t raw = g_axis.fbdk.uAngleRaw;
    int32_t delta =
        SpeedAutoTune_GetWrappedDelta(raw, s_speedAutoTune.previousRaw);
    int64_t absolutePosition;

    /* 将按单圈回绕的编码器位置展开为相对实验起点的有符号累计位移。 */
    s_speedAutoTune.previousRaw = raw;
    s_speedAutoTune.unwrappedCounts += (int64_t)delta;
    absolutePosition = (s_speedAutoTune.unwrappedCounts >= 0) ?
                       s_speedAutoTune.unwrappedCounts :
                      -s_speedAutoTune.unwrappedCounts;

    if (absolutePosition >
        (int64_t)(SPEED_AUTOTUNE_MAX_TRAVEL_REV * (float)ENCODER_COUNT))
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_TRAVEL_LIMIT);
        return;
    }

    /*
     * 在固定短窗口内对位置增量求平均。该估算器没有低速死区，适合 0.5 Hz 小幅运动
     * 辨识；窗口平均值同时作为独立于正常测速链路的超速保护依据。
     */
    s_speedAutoTune.guardIqSumA +=
        FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE;
    s_speedAutoTune.guardWindowSamples++;
    if (s_speedAutoTune.guardWindowSamples >= SPEED_AUTOTUNE_GUARD_SAMPLES)
    {
        float periodS =
            (float)s_speedAutoTune.guardWindowSamples /
            (float)SPEED_CONTROL_RATE;
        int64_t countDelta = s_speedAutoTune.unwrappedCounts -
                             s_speedAutoTune.guardWindowStartCounts;

        s_speedAutoTune.guardMechanicalRpm =
            ((float)countDelta / (float)ENCODER_COUNT) *
            (60.0f / periodS);
        s_speedAutoTune.guardMeanIqA =
            s_speedAutoTune.guardIqSumA /
            (float)s_speedAutoTune.guardWindowSamples;
        s_speedAutoTune.guardValid = true;
        s_speedAutoTune.guardWindowStartCounts =
            s_speedAutoTune.unwrappedCounts;
        s_speedAutoTune.guardWindowSamples = 0U;
        s_speedAutoTune.guardIqSumA = 0.0f;

        if (fabsf(s_speedAutoTune.guardMechanicalRpm) >
            SPEED_AUTOTUNE_MAX_RPM)
        {
            SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_OVERSPEED);
            return;
        }
    }

    s_speedAutoTune.totalTicks++;
}

/*
 * 自适应单正弦阶段主流程：
 *
 * 1. 每个速度控制周期把独立窗口转速投影到 0.5 Hz 的 sin/cos 基函数。
 * 2. 每完成一个完整周期，用 2/N 恢复相干速度基波幅值。
 * 3. 电流幅值切换后的第一个周期只用于稳定，不参与接受判定。
 * 4. 若速度幅值达到常规门限，则锁定当前电流并进入三音稳定阶段。
 * 5. 若响应不足，则把电流乘以固定步进比；到达授权上限后允许低机动性回退门限。
 * 6. 若授权上限下仍无最小有效响应，或搜索次数耗尽，则安全失败。
 */
static void SpeedAutoTune_UpdateSineAdapt(void)
{
    uint32_t periodTicks = SpeedAutoTune_GetSinePeriodTicks();
    uint32_t sampleTick = s_speedAutoTune.stageTicks - 1U;
    float phase = 2.0f * SPEED_AUTOTUNE_PI *
                  SPEED_AUTOTUNE_SINE_FREQUENCY_HZ *
                  ((float)sampleTick / (float)SPEED_CONTROL_RATE);
    float sine = sinf(phase);
    float cosine = cosf(phase);
    float rpm = s_speedAutoTune.guardMechanicalRpm;

    /*
     * 只使用与指令正弦相干的速度分量。单周期最大值减最小值容易被启动瞬态、编码器
     * 量化或单个齿槽峰值放大，从而过早停止电流搜索。同步解调能够抑制直流漂移和
     * 大部分非相关谐波，使幅值选择建立在持续、可用的激励响应上。
     */
    s_speedAutoTune.sineAdaptSpeedSinSum += rpm * sine;
    s_speedAutoTune.sineAdaptSpeedCosSum += rpm * cosine;
    s_speedAutoTune.sineAdaptSamples++;

    if (s_speedAutoTune.stageTicks >= periodTicks)
    {
        float inverseSamples =
            1.0f / (float)s_speedAutoTune.sineAdaptSamples;
        float speedSin = 2.0f *
            s_speedAutoTune.sineAdaptSpeedSinSum * inverseSamples;
        float speedCos = 2.0f *
            s_speedAutoTune.sineAdaptSpeedCosSum * inverseSamples;
        float speedAmplitudeRpm = sqrtf(speedSin * speedSin +
                                        speedCos * speedCos);

        g_speedAutoTuneResult.speedAmplitudeRpm = speedAmplitudeRpm;

        /*
         * 每次电流幅值变化后丢弃第一个完整周期。虽然激励在过零点切换，但该周期仍
         * 包含电流幅值变化后的机械过渡；下一个完整周期才用于接受当前幅值或继续增大。
         */
        s_speedAutoTune.sineAdaptDwellCycles++;
        if (s_speedAutoTune.sineAdaptDwellCycles <=
            SPEED_AUTOTUNE_SINE_ADAPT_DISCARD_CYCLES)
        {
            s_speedAutoTune.stageTicks = 0U;
            SpeedAutoTune_ResetSineAdaptCycle();
            s_speedAutoTune.sineCommandA =
                SpeedAutoTune_GetCenteredSineCommand(0U, 1.0f);
            return;
        }

        s_speedAutoTune.sineAdaptCycles++;
        if (isfinite(speedAmplitudeRpm) &&
            speedAmplitudeRpm >=
            SPEED_AUTOTUNE_SINE_ADAPT_TARGET_RPM)
        {
            SpeedAutoTune_UpdateMobilityProfile(speedAmplitudeRpm);
            s_speedAutoTune.sineCommandA =
                SpeedAutoTune_GetCenteredSineCommand(0U, 1.0f);
            SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SINE_SETTLE);
            return;
        }

        {
            float nextCurrentA = SpeedAutoTune_Clamp(
                s_speedAutoTune.sineCurrentA *
                    SPEED_AUTOTUNE_SINE_CURRENT_STEP_RATIO,
                SPEED_AUTOTUNE_SINE_INITIAL_CURRENT_A,
                g_speedAutoTuneResult.testCurrentA);

            if (nextCurrentA <= s_speedAutoTune.sineCurrentA * 1.001f)
            {
                /*
                 * 低机动性对象可能产生很干净但幅值明显较小的速度基波。不能直接降低
                 * 常规 4 rpm 门限；只有配置的电流上限本身已经完整测试后，才接受较小
                 * 响应。即便如此，仍必须通过后续四周期相干性检查和有界 Kp 验证，
                 * PI 才能最终提交。
                 */
                if (isfinite(speedAmplitudeRpm) &&
                    speedAmplitudeRpm >=
                        SPEED_AUTOTUNE_CEILING_MIN_RESPONSE_RPM)
                {
                    s_speedAutoTune.sineCeilingResponseAccepted = true;
                    SpeedAutoTune_UpdateMobilityProfile(
                        speedAmplitudeRpm);
                    printf("[SPEED_AUTOTUNE] accepting low-response plant at "
                           "current ceiling: speedAmp=%.3f rpm, "
                           "minimum=%.3f rpm\n",
                           speedAmplitudeRpm,
                           SPEED_AUTOTUNE_CEILING_MIN_RESPONSE_RPM);
                    s_speedAutoTune.sineCommandA =
                        SpeedAutoTune_GetCenteredSineCommand(0U, 1.0f);
                    SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SINE_SETTLE);
                    return;
                }

                SpeedAutoTune_Fail(
                    SPEED_AUTOTUNE_ERROR_EXCITATION_TOO_SMALL);
                return;
            }

            /*
             * 当授权上限为 0.5 A 时，固定 1.5 倍搜索在第八个有效周期只到 0.342 A。
             * 因此允许最后一次直接跳到精确安全上限并测量完整周期，否则高摩擦或高负载
             * 对象会在尚未使用调用方授权电流前就被拒绝。中间幅值仍受正常周期上限
             * 约束，所以该例外不会形成无界搜索。
             */
            if (s_speedAutoTune.sineAdaptCycles >=
                    SPEED_AUTOTUNE_SINE_MAX_ADAPT_CYCLES &&
                nextCurrentA <
                    g_speedAutoTuneResult.testCurrentA * 0.999f)
            {
                SpeedAutoTune_Fail(
                    SPEED_AUTOTUNE_ERROR_EXCITATION_TOO_SMALL);
                return;
            }

            /* 幅值只在 sin(2*pi*n)=0 的过零点变化，因此 Iq 指令保持连续。 */
            s_speedAutoTune.sineCurrentA = nextCurrentA;
            s_speedAutoTune.stageTicks = 0U;
            s_speedAutoTune.sineAdaptDwellCycles = 0U;
            SpeedAutoTune_ResetSineAdaptCycle();
            s_speedAutoTune.sineCommandA =
                SpeedAutoTune_GetCenteredSineCommand(0U, 1.0f);
            return;
        }
    }

    s_speedAutoTune.sineCommandA = SpeedAutoTune_GetCenteredSineCommand(
        s_speedAutoTune.stageTicks, 1.0f);
}

/*
 * 三音稳定阶段把激励强度从 0 按 SmoothStep 淡入到 1，并等待整数个基波周期。
 * 该阶段不累计 FRF，目的是让单正弦搜索末态和三音稳态之间的机械过渡完全衰减。
 */
static void SpeedAutoTune_UpdateSineSettle(void)
{
    uint32_t requiredTicks = SpeedAutoTune_GetSinePeriodTicks() *
                             SPEED_AUTOTUNE_SINE_SETTLE_CYCLES;

    if (s_speedAutoTune.stageTicks >= requiredTicks)
    {
        SpeedAutoTune_ResetSineMeasurement();
        s_speedAutoTune.sineCommandA =
            SpeedAutoTune_GetCenteredSineCommand(0U, 1.0f);
        SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SINE_MEASURE);
        return;
    }

    s_speedAutoTune.sineCommandA = SpeedAutoTune_GetCenteredSineCommand(
        s_speedAutoTune.stageTicks,
        SpeedAutoTune_SmoothStep(
            (float)s_speedAutoTune.stageTicks / (float)requiredTicks));
}

static void SpeedAutoTune_AccumulateSineSample(uint32_t sampleTick)
{
    float timeS = (float)sampleTick / (float)SPEED_CONTROL_RATE;
    float iqA = FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE;
    float speedElectricalHz = s_speedAutoTune.guardMechanicalRpm *
                              (float)POLE_PAIR_NUM / 60.0f;

    /*
     * 同步解调把每个测量信号投影到各注入频率对应的 sin/cos 基函数。在整数个基波
     * 周期内，直流分量、非相关编码器波纹和大多数齿槽谐波会积分趋近于零。计算幅值
     * 时再统一乘以 2/N，得到各音调的峰值幅值。
     */
    for (uint32_t tone = 0U;
         tone < SPEED_AUTOTUNE_MULTISINE_TONE_COUNT;
         tone++)
    {
        float phase = 2.0f * SPEED_AUTOTUNE_PI *
                      s_multisineFrequencyHz[tone] * timeS +
                      s_multisinePhaseRad[tone];
        float sine = sinf(phase);
        float cosine = cosf(phase);

        s_speedAutoTune.iqSinSum[tone] += iqA * sine;
        s_speedAutoTune.iqCosSum[tone] += iqA * cosine;
        s_speedAutoTune.commandSinSum[tone] +=
            s_speedAutoTune.sineCommandA * sine;
        s_speedAutoTune.commandCosSum[tone] +=
            s_speedAutoTune.sineCommandA * cosine;
        s_speedAutoTune.speedSinSum[tone] += speedElectricalHz * sine;
        s_speedAutoTune.speedCosSum[tone] += speedElectricalHz * cosine;
    }
    s_speedAutoTune.iqSum += iqA;
    s_speedAutoTune.speedSum += speedElectricalHz;
    s_speedAutoTune.iqSquareSum += iqA * iqA;
    s_speedAutoTune.speedSquareSum +=
        speedElectricalHz * speedElectricalHz;
    s_speedAutoTune.sineSamples++;

    if (PIDREGDQX_CURRENT_getClipped(&g_axis.currCtrl.pidIdIq))
    {
        s_speedAutoTune.sineClippedSamples++;
    }
}

/*
 * 三音测量阶段持续使用固定幅值，不再调整电流。每个速度控制周期先累计本拍对应的
 * 指令 Iq、实测 Iq、速度相量与能量统计；达到配置的完整基波周期数后停止采样，进入
 * 不计入模型的平滑淡出制动阶段。
 */
static void SpeedAutoTune_UpdateSineMeasure(void)
{
    uint32_t requiredTicks = SpeedAutoTune_GetSinePeriodTicks() *
                             SPEED_AUTOTUNE_SINE_MEASURE_CYCLES;

    /* 调用本函数前 stageTicks 已自增，本拍实际施加的激励样本索引为 n-1。 */
    SpeedAutoTune_AccumulateSineSample(s_speedAutoTune.stageTicks - 1U);

    if (s_speedAutoTune.stageTicks >= requiredTicks)
    {
        /* 制动先执行一个不参与测量的三音平滑淡出周期，避免激励突然归零。 */
        s_speedAutoTune.sineBrakeDirection = 0;
        SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SINE_BRAKE);
        return;
    }

    s_speedAutoTune.sineCommandA = SpeedAutoTune_GetCenteredSineCommand(
        s_speedAutoTune.stageTicks, 1.0f);
}

/*
 * 三音制动分两段：先用一个完整基波周期平滑淡出外部激励，再只保留回中 P 阻尼使
 * 转子停下。转速进入停车窗口并连续保持规定时间后才进入 SOLVE；若转速重新超过
 * 复位门限则清零停车计时，超过总制动时限则报阶段超时。
 */
static void SpeedAutoTune_UpdateSineBrake(void)
{
    uint32_t fadeTicks = SpeedAutoTune_GetSinePeriodTicks();
    float rpm = s_speedAutoTune.guardMechanicalRpm;
    int8_t direction = (rpm > SPEED_AUTOTUNE_SINE_STOP_RPM) ? 1 :
                       (rpm < -SPEED_AUTOTUNE_SINE_STOP_RPM) ? -1 : 0;

    if (s_speedAutoTune.stageTicks <= fadeTicks)
    {
        float fadeProgress = (float)s_speedAutoTune.stageTicks /
                             (float)fadeTicks;
        float forcingScale = 1.0f -
                             SpeedAutoTune_SmoothStep(fadeProgress);

        s_speedAutoTune.sineCommandA =
            SpeedAutoTune_GetCenteredSineCommand(
                s_speedAutoTune.stageTicks, forcingScale);
        if (s_speedAutoTune.stageTicks == fadeTicks)
        {
            PIDREGDQX_CURRENT_setUiQ_pu(
                &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
        }
        return;
    }

    /* 转速每次穿越零点都清除电流调节器积分，避免旧方向的阻尼转矩残留。 */
    if (direction != 0 &&
        s_speedAutoTune.sineBrakeDirection != 0 &&
        direction != s_speedAutoTune.sineBrakeDirection)
    {
        PIDREGDQX_CURRENT_setUiQ_pu(
            &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
    }
    if (direction != 0)
    {
        s_speedAutoTune.sineBrakeDirection = direction;
    }

    s_speedAutoTune.sineCommandA = SpeedAutoTune_GetCenteredSineCommand(
        s_speedAutoTune.stageTicks, 0.0f);

    if (fabsf(rpm) <= SPEED_AUTOTUNE_SINE_STOP_RPM)
    {
        /* 进入停车窗口后不允许保留任何积分转矩。 */
        s_speedAutoTune.sineCommandA = 0.0f;
        PIDREGDQX_CURRENT_setUiQ_pu(
            &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
        s_speedAutoTune.stopStableTicks++;
        if (s_speedAutoTune.stopStableTicks >=
            SPEED_AUTOTUNE_MS_TO_TICKS(SPEED_AUTOTUNE_STOP_STABLE_MS))
        {
            s_speedAutoTune.sineCommandA = 0.0f;
            PIDREGDQX_CURRENT_setUiQ_pu(
                &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
            SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SOLVE);
            return;
        }
    }
    else if (fabsf(rpm) >= SPEED_AUTOTUNE_SINE_STOP_RESET_RPM)
    {
        s_speedAutoTune.stopStableTicks = 0U;
    }

    if (s_speedAutoTune.stageTicks >=
        SPEED_AUTOTUNE_MS_TO_TICKS(
            SPEED_AUTOTUNE_SINE_BRAKE_TIMEOUT_MS))
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_STAGE_TIMEOUT);
    }
}

static void SpeedAutoTune_AcceptKpProbe(void)
{
    float kiScale = s_speedAutoTune.kpProbeGain /
                    g_speedAutoTuneResult.modelKp_A_per_eHz;
    float finalKiScale = SpeedAutoTune_BlendProfile(
        1.0f, SPEED_AUTOTUNE_LOW_MOBILITY_KI_SCALE);

    /*
     * 用实测验证 Kp 替换模型 Kp 时保持模型 PI 零点：先按 acceptedKp/modelKp 对两个
     * 增益同比缩放。低机动性对象的表观低速响应包含惯性模型无法可靠描述的摩擦，
     * 因此再将最终 Ki 向 LOW_MOBILITY_KI_SCALE 所规定的较低比例收缩。
     */
    g_speedAutoTuneResult.probedKp_A_per_eHz =
        s_speedAutoTune.kpProbeGain;
    g_speedAutoTuneResult.kp_A_per_eHz =
        s_speedAutoTune.kpProbeGain;
    g_speedAutoTuneResult.ki_A_per_eHz_s =
        g_speedAutoTuneResult.modelKi_A_per_eHz_s * kiScale *
        finalKiScale;
    s_speedAutoTune.kpProbeIntegralA = 0.0f;
    s_speedAutoTune.kpBrakeStartRpm =
        SpeedAutoTune_GetKpFeedbackRpm();
    s_speedAutoTune.kpBrakeDirection = 0;
    PIDREGDQX_CURRENT_setUiQ_pu(
        &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
    SpeedAutoTune_EnterState(SPEED_AUTOTUNE_KP_PROBE_BRAKE);
}

static void SpeedAutoTune_RecordKpWindow(float meanRpm,
                                         float minimumRpm,
                                         float maximumRpm,
                                         float rmsErrorRpm,
                                         bool passed)
{
    uint32_t index = s_speedAutoTune.kpWindowCount;

    if (index >= SPEED_AUTOTUNE_KP_PROBE_MAX_WINDOWS)
    {
        return;
    }

    s_speedAutoTune.kpWindowGain[index] =
        s_speedAutoTune.kpProbeGain;
    s_speedAutoTune.kpWindowTargetRpm[index] =
        s_speedAutoTune.kpProbeTargetRpm;
    s_speedAutoTune.kpWindowMeanRpm[index] = meanRpm;
    s_speedAutoTune.kpWindowMinimumRpm[index] = minimumRpm;
    s_speedAutoTune.kpWindowMaximumRpm[index] = maximumRpm;
    s_speedAutoTune.kpWindowRmsErrorRpm[index] = rmsErrorRpm;
    s_speedAutoTune.kpWindowPassed[index] = passed;
    s_speedAutoTune.kpWindowCount++;
}

static void SpeedAutoTune_ResetKpCandidate(void)
{
    s_speedAutoTune.kpCandidateWindows = 0U;
    s_speedAutoTune.kpCandidateStrict = true;
    s_speedAutoTune.kpCandidateSafe = true;
    s_speedAutoTune.kpCandidateWorstRmsRpm = 0.0f;
    s_speedAutoTune.kpCandidateMeanRpm = 0.0f;
    s_speedAutoTune.kpCandidateMinimumRpm = 0.0f;
    s_speedAutoTune.kpCandidateMaximumRpm = 0.0f;
    s_speedAutoTune.kpProbePasses = 0U;
}

static void SpeedAutoTune_SelectKp(float gain,
                                   float meanRpm,
                                   float minimumRpm,
                                   float maximumRpm,
                                   float rmsErrorRpm,
                                   bool fallback)
{
    s_speedAutoTune.kpProbeGain = gain;
    g_speedAutoTuneResult.kpProbeMeanRpm = meanRpm;
    g_speedAutoTuneResult.kpProbeMinimumRpm = minimumRpm;
    g_speedAutoTuneResult.kpProbeMaximumRpm = maximumRpm;
    g_speedAutoTuneResult.kpProbeRmsErrorRpm = rmsErrorRpm;
    g_speedAutoTuneResult.kpSelectedByFallback = fallback;
    SpeedAutoTune_AcceptKpProbe();
}

static void SpeedAutoTune_ScheduleKpRamp(float nextTargetRpm)
{
    /*
     * 每完成一个验证窗口就反转目标方向。SmoothStep 过渡让转子往返经过同一位移区域，
     * 既限制累计位移，又能同时验证两个方向上的齿槽和摩擦影响。
     */
    s_speedAutoTune.kpRampStartRpm =
        s_speedAutoTune.kpProbeTargetRpm;
    s_speedAutoTune.kpRampEndRpm = nextTargetRpm;
    s_speedAutoTune.kpProbeTargetRpm = nextTargetRpm;
    /* 每次换向都从零重新建立该方向的摩擦偏置，禁止沿用上一方向的积分。 */
    s_speedAutoTune.kpProbeIntegralA = 0.0f;
    SpeedAutoTune_ResetKpProbeMeasurement();
    SpeedAutoTune_EnterState(SPEED_AUTOTUNE_KP_RAMP);
}

/*
 * 单个 Kp 候选的双向低速验证流程：
 *
 * 1. 先等待 settleTicks，只在随后 measureTicks 内累计卡尔曼速度统计。
 * 2. 根据目标方向把速度统一转换为正值，计算均值、极值和 RMS 跟踪误差。
 * 3. 分别评价严格跟踪窗口和宽松安全窗口，并把正、反两个方向按逻辑与合并。
 * 4. 完成第一个方向后，用同一 Kp 平滑换向；两个方向完成后才计算候选效用。
 * 5. 保存效用最高的安全候选。未到终止条件时按比例提高 Kp，重新验证两个方向。
 * 6. 达到 Kp/次数上限或连续多个候选不再改善时，提交最佳安全候选；没有安全候选
 *    则报告 Kp 探测失败。
 */
static void SpeedAutoTune_UpdateKpProbe(void)
{
    uint32_t settleTicks = SPEED_AUTOTUNE_MS_TO_TICKS(
        SpeedAutoTune_GetKpSettleMs());
    uint32_t measureTicks = SPEED_AUTOTUNE_MS_TO_TICKS(
        SpeedAutoTune_GetKpMeasureMs());
    float targetRpm = SpeedAutoTune_GetKpValidationTargetRpm();
    /*
     * 将正、反两个目标方向都转换到统一的正向坐标系，使同一组均值、最小值、最大值
     * 和 RMS 门限能够完全一致地评价两个方向；方向不对称的摩擦会表现为窗口失败。
     */
    float direction = (s_speedAutoTune.kpProbeTargetRpm >= 0.0f) ?
                      1.0f : -1.0f;
    float rpm = direction * SpeedAutoTune_GetKpFeedbackRpm();
    float speedErrorRpm = rpm - targetRpm;

    if (s_speedAutoTune.stageTicks == settleTicks)
    {
        SpeedAutoTune_ResetKpProbeMeasurement();
    }

    if (s_speedAutoTune.stageTicks >= settleTicks)
    {
        s_speedAutoTune.kpProbeSpeedSumRpm += rpm;
        s_speedAutoTune.kpProbeErrorSquareSum +=
            speedErrorRpm * speedErrorRpm;
        if (rpm < s_speedAutoTune.kpProbeMinimumRpm)
        {
            s_speedAutoTune.kpProbeMinimumRpm = rpm;
        }
        if (rpm > s_speedAutoTune.kpProbeMaximumRpm)
        {
            s_speedAutoTune.kpProbeMaximumRpm = rpm;
        }
        s_speedAutoTune.kpProbeSamples++;
    }

    if (s_speedAutoTune.stageTicks < settleTicks + measureTicks)
    {
        return;
    }

    if (s_speedAutoTune.kpProbeSamples > 0U)
    {
        float meanRpm = s_speedAutoTune.kpProbeSpeedSumRpm /
                        (float)s_speedAutoTune.kpProbeSamples;
        float rmsErrorRpm = sqrtf(
            s_speedAutoTune.kpProbeErrorSquareSum /
            (float)s_speedAutoTune.kpProbeSamples);
        /*
         * strictWindow 是期望达到的跟踪质量门限；下面的 safeWindow 有意放宽，只判断
         * 响应是否仍然有界、能否作为安全回退候选。同一个 Kp 必须在正反两个窗口都
         * 安全；严格状态同样对两个方向取逻辑与，禁止单个有利方向掩盖另一方向的问题。
         */
        bool strictWindow =
            meanRpm >= targetRpm *
                       SPEED_AUTOTUNE_KP_PROBE_MIN_MEAN_RATIO &&
            meanRpm <= targetRpm *
                       SPEED_AUTOTUNE_KP_PROBE_MAX_MEAN_RATIO &&
            s_speedAutoTune.kpProbeMinimumRpm >=
                targetRpm *
                SPEED_AUTOTUNE_KP_PROBE_MIN_SPEED_RATIO &&
            s_speedAutoTune.kpProbeMaximumRpm <=
                targetRpm *
                SPEED_AUTOTUNE_KP_PROBE_MAX_SPEED_RATIO &&
            rmsErrorRpm <= targetRpm *
                           SPEED_AUTOTUNE_KP_PROBE_MAX_RMS_ERROR_RATIO;
        float safeMinMeanRatio = SpeedAutoTune_BlendProfile(
            0.70f, SPEED_AUTOTUNE_LOW_MOBILITY_MIN_MEAN_RATIO);
        float safeMaxMeanRatio = SpeedAutoTune_BlendProfile(
            1.30f, SPEED_AUTOTUNE_LOW_MOBILITY_MAX_MEAN_RATIO);
        float safeMinSpeedRatio = SpeedAutoTune_BlendProfile(
            0.0f, SPEED_AUTOTUNE_LOW_MOBILITY_MIN_SPEED_RATIO);
        float safeMaxSpeedRatio = SpeedAutoTune_BlendProfile(
            2.0f, SPEED_AUTOTUNE_LOW_MOBILITY_MAX_SPEED_RATIO);
        float safeMaxRmsRatio = SpeedAutoTune_BlendProfile(
            SPEED_AUTOTUNE_KP_PROBE_FALLBACK_RMS_RATIO,
            SPEED_AUTOTUNE_LOW_MOBILITY_MAX_RMS_RATIO);
        bool safeWindow =
            meanRpm >= targetRpm * safeMinMeanRatio &&
            meanRpm <= targetRpm * safeMaxMeanRatio &&
            s_speedAutoTune.kpProbeMinimumRpm >=
                targetRpm * safeMinSpeedRatio &&
            s_speedAutoTune.kpProbeMaximumRpm <=
                targetRpm * safeMaxSpeedRatio &&
            rmsErrorRpm <= targetRpm * safeMaxRmsRatio;

        g_speedAutoTuneResult.kpProbeMeanRpm = meanRpm;
        g_speedAutoTuneResult.kpProbeMinimumRpm =
            s_speedAutoTune.kpProbeMinimumRpm;
        g_speedAutoTuneResult.kpProbeMaximumRpm =
            s_speedAutoTune.kpProbeMaximumRpm;
        g_speedAutoTuneResult.kpProbeRmsErrorRpm = rmsErrorRpm;

        SpeedAutoTune_RecordKpWindow(
            meanRpm,
            s_speedAutoTune.kpProbeMinimumRpm,
            s_speedAutoTune.kpProbeMaximumRpm,
            rmsErrorRpm,
            strictWindow);

        s_speedAutoTune.kpCandidateWindows++;
        s_speedAutoTune.kpCandidateStrict =
            s_speedAutoTune.kpCandidateStrict && strictWindow;
        s_speedAutoTune.kpCandidateSafe =
            s_speedAutoTune.kpCandidateSafe && safeWindow;
        if (rmsErrorRpm >=
            s_speedAutoTune.kpCandidateWorstRmsRpm)
        {
            s_speedAutoTune.kpCandidateWorstRmsRpm = rmsErrorRpm;
            s_speedAutoTune.kpCandidateMeanRpm = meanRpm;
            s_speedAutoTune.kpCandidateMinimumRpm =
                s_speedAutoTune.kpProbeMinimumRpm;
            s_speedAutoTune.kpCandidateMaximumRpm =
                s_speedAutoTune.kpProbeMaximumRpm;
        }

        if (strictWindow)
        {
            s_speedAutoTune.kpProbePasses++;
        }

        if (s_speedAutoTune.kpCandidateWindows <
            SPEED_AUTOTUNE_KP_PROBE_REQUIRED_PASSES)
        {
            /* 比较候选之前，必须用同一个 Kp 完成相反方向的验证窗口。 */
            SpeedAutoTune_ScheduleKpRamp(
                -s_speedAutoTune.kpProbeTargetRpm);
            return;
        }

        /*
         * 候选效用随对象机动性连续变化，两个端点有意采用不同目标：
         *
         *   score=0（高机动性）：utility = -RMS/target + strict_bonus
         *   score=1（低机动性）：utility = Kp/Kp_max
         *
         * 高机动性对象优先选择严格通过且 RMS 最低的候选；受摩擦或折算负载主导的对象
         * 优先选择仍通过宽松安全窗口的最高刚度候选；中间分数在两者之间连续折中，
         * 无需先判断电机类型。这里的 utility 只用于 Kp 候选排序，不是速度控制器积分。
         */
        float mobilityScore = g_speedAutoTuneResult.lowMobilityScore;
        float gainNorm = s_speedAutoTune.kpProbeGain /
            fmaxf(s_speedAutoTune.kpProbeMaximumGain, 1.0e-6f);
        float rmsNorm = s_speedAutoTune.kpCandidateWorstRmsRpm /
            fmaxf(targetRpm, 1.0e-6f);
        float candidateUtility = mobilityScore * gainNorm -
            (1.0f - mobilityScore) * rmsNorm +
            (s_speedAutoTune.kpCandidateStrict ?
                (1.0f - mobilityScore) : 0.0f);
        bool utilityUpgrade = candidateUtility >
            s_speedAutoTune.kpBestUtility + 1.0e-6f;

        if (s_speedAutoTune.kpCandidateSafe &&
            (!s_speedAutoTune.kpBestValid || utilityUpgrade))
        {
            s_speedAutoTune.kpBestValid = true;
            s_speedAutoTune.kpBestStrict =
                s_speedAutoTune.kpCandidateStrict;
            s_speedAutoTune.kpBestGain =
                s_speedAutoTune.kpProbeGain;
            s_speedAutoTune.kpBestWorstRmsRpm =
                s_speedAutoTune.kpCandidateWorstRmsRpm;
            s_speedAutoTune.kpBestMeanRpm =
                s_speedAutoTune.kpCandidateMeanRpm;
            s_speedAutoTune.kpBestMinimumRpm =
                s_speedAutoTune.kpCandidateMinimumRpm;
            s_speedAutoTune.kpBestMaximumRpm =
                s_speedAutoTune.kpCandidateMaximumRpm;
            s_speedAutoTune.kpBestUtility = candidateUtility;
            s_speedAutoTune.kpWorseCandidates = 0U;
        }
        else if (s_speedAutoTune.kpBestValid)
        {
            s_speedAutoTune.kpWorseCandidates++;
        }
    }

    if (s_speedAutoTune.kpProbeAttempts >=
            SPEED_AUTOTUNE_KP_PROBE_MAX_ATTEMPTS ||
        s_speedAutoTune.kpProbeGain >=
            s_speedAutoTune.kpProbeMaximumGain * 0.999f ||
        s_speedAutoTune.kpWorseCandidates >=
            SPEED_AUTOTUNE_KP_PROBE_STOP_AFTER_WORSE)
    {
        if (s_speedAutoTune.kpBestValid)
        {
            s_speedAutoTune.kpProbePasses =
                s_speedAutoTune.kpBestStrict ?
                SPEED_AUTOTUNE_KP_PROBE_REQUIRED_PASSES : 0U;
            SpeedAutoTune_SelectKp(
                s_speedAutoTune.kpBestGain,
                s_speedAutoTune.kpBestMeanRpm,
                s_speedAutoTune.kpBestMinimumRpm,
                s_speedAutoTune.kpBestMaximumRpm,
                s_speedAutoTune.kpBestWorstRmsRpm,
                !s_speedAutoTune.kpBestStrict);
            return;
        }

        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_KP_PROBE);
        return;
    }

    {
        float nextKp = s_speedAutoTune.kpProbeGain *
                       SpeedAutoTune_GetKpStepRatio();

        if (nextKp > s_speedAutoTune.kpProbeMaximumGain)
        {
            nextKp = s_speedAutoTune.kpProbeMaximumGain;
        }
        if (nextKp <= s_speedAutoTune.kpProbeGain * 1.001f)
        {
            SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_KP_PROBE);
            return;
        }

        s_speedAutoTune.kpProbeGain = nextKp;
        s_speedAutoTune.kpProbeAttempts++;
        SpeedAutoTune_ResetKpCandidate();
        SpeedAutoTune_ScheduleKpRamp(
            -s_speedAutoTune.kpProbeTargetRpm);
    }
}

static void SpeedAutoTune_UpdateKpProbeBrake(void)
{
    float rpm = s_speedAutoTune.guardMechanicalRpm;
    float stopRpm = SpeedAutoTune_GetKpBrakeStopRpm();
    float resetRpm = SpeedAutoTune_GetKpBrakeResetRpm();

    if (fabsf(rpm) <= stopRpm)
    {
        s_speedAutoTune.stopStableTicks++;
        if (s_speedAutoTune.stopStableTicks >=
            SPEED_AUTOTUNE_MS_TO_TICKS(SPEED_AUTOTUNE_STOP_STABLE_MS))
        {
            PIDREG_SPEED_setKp_si(
                &g_axis.speedCtrl.PIDSpeed,
                g_speedAutoTuneResult.kp_A_per_eHz);
            PIDREG_SPEED_setKi_si(
                &g_axis.speedCtrl.PIDSpeed,
                g_speedAutoTuneResult.ki_A_per_eHz_s);
            PIDREGDQX_CURRENT_setUiQ_pu(
                &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
            g_speedAutoTuneResult.valid = true;
            SpeedAutoTune_EnterState(SPEED_AUTOTUNE_FINISH);
            return;
        }
    }
    else if (fabsf(rpm) >= resetRpm)
    {
        s_speedAutoTune.stopStableTicks = 0U;
    }

    if (s_speedAutoTune.stageTicks >=
        SPEED_AUTOTUNE_MS_TO_TICKS(
            SpeedAutoTune_GetKpBrakeTimeoutMs()))
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_STAGE_TIMEOUT);
    }
}

/*
 * 速度控制频率下的状态调度器。stageTicks 在分派前自增，因此各阶段看到的第一拍为
 * 1；需要与刚刚施加的激励样本对齐时使用 stageTicks-1。SOLVE、FOC 恢复、FINISH 和
 * FAULT 由 PWM 高频入口单独处理，这里只调度需要机械周期计时的状态。
 */
static void SpeedAutoTune_UpdateStage(void)
{
    s_speedAutoTune.stageTicks++;

    switch (g_speedAutoTuneResult.state)
    {
        case SPEED_AUTOTUNE_SETTLE:
            if (s_speedAutoTune.stageTicks >=
                SPEED_AUTOTUNE_MS_TO_TICKS(SPEED_AUTOTUNE_SETTLE_MS))
            {
                s_speedAutoTune.sineAdaptCycles = 0U;
                s_speedAutoTune.sineAdaptDwellCycles = 0U;
                SpeedAutoTune_ResetSineAdaptCycle();
                SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SINE_ADAPT);
            }
            break;

        case SPEED_AUTOTUNE_SINE_ADAPT:
            SpeedAutoTune_UpdateSineAdapt();
            break;

        case SPEED_AUTOTUNE_SINE_SETTLE:
            SpeedAutoTune_UpdateSineSettle();
            break;

        case SPEED_AUTOTUNE_SINE_MEASURE:
            SpeedAutoTune_UpdateSineMeasure();
            break;

        case SPEED_AUTOTUNE_SINE_BRAKE:
            SpeedAutoTune_UpdateSineBrake();
            break;

        case SPEED_AUTOTUNE_KP_RAMP:
            if (s_speedAutoTune.stageTicks >=
                SPEED_AUTOTUNE_MS_TO_TICKS(
                    SpeedAutoTune_GetKpRampMs()))
            {
                SpeedAutoTune_ResetKpProbeMeasurement();
                SpeedAutoTune_EnterState(SPEED_AUTOTUNE_KP_PROBE);
            }
            break;

        case SPEED_AUTOTUNE_KP_PROBE:
            SpeedAutoTune_UpdateKpProbe();
            break;

        case SPEED_AUTOTUNE_KP_PROBE_BRAKE:
            SpeedAutoTune_UpdateKpProbeBrake();
            break;

        default:
            break;
    }
}

static float SpeedAutoTune_GetKpTargetRpm(void)
{
    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_KP_RAMP)
    {
        uint32_t rampTicks = SPEED_AUTOTUNE_MS_TO_TICKS(
            SpeedAutoTune_GetKpRampMs());
        float progress = (rampTicks > 0U) ?
            (float)s_speedAutoTune.stageTicks / (float)rampTicks : 1.0f;

        float blend = SpeedAutoTune_SmoothStep(progress);

        return s_speedAutoTune.kpRampStartRpm +
               (s_speedAutoTune.kpRampEndRpm -
                s_speedAutoTune.kpRampStartRpm) * blend;
    }

    return s_speedAutoTune.kpProbeTargetRpm;
}

static float SpeedAutoTune_GetKpProbeIq(void)
{
    float speedErrorElectricalHz =
        (SpeedAutoTune_GetKpTargetRpm() -
         SpeedAutoTune_GetKpFeedbackRpm()) *
        (float)POLE_PAIR_NUM / 60.0f;
    float proportionalA = s_speedAutoTune.kpProbeGain *
                          speedErrorElectricalHz;
    /*
     * 探测积分随 lowMobilityScore 的方向如下：
     *
     *   lowMobilityScore=0 -> probeKiScale=0，仅使用 P 验证
     *   lowMobilityScore=1 -> probeKiScale=LOW_MOBILITY_KI_SCALE
     *
     * 因此高机动性对象获得最小积分作用，也就是完全不使用积分。score=0 时整个积分
     * 分支都会跳过，所以下面计算出的积分启用速度在该端点不起作用。
     */
    float probeKiScale = g_speedAutoTuneResult.lowMobilityScore *
                         SPEED_AUTOTUNE_LOW_MOBILITY_KI_SCALE;

    if (probeKiScale > 1.0e-6f)
    {
        float kiScale = s_speedAutoTune.kpProbeGain /
                        g_speedAutoTuneResult.modelKp_A_per_eHz;
        float probeKi = g_speedAutoTuneResult.modelKi_A_per_eHz_s *
                        kiScale * probeKiScale;
        float targetRpm = SpeedAutoTune_GetKpTargetRpm();
        float direction = (targetRpm >= 0.0f) ? 1.0f : -1.0f;
        float directedRpm = direction *
                            SpeedAutoTune_GetKpFeedbackRpm();
        float integralLimitA = g_speedAutoTuneResult.testCurrentA *
            SPEED_AUTOTUNE_LOW_MOBILITY_I_LIMIT_RATIO;
        float integralEnableRpm =
            g_speedAutoTuneResult.lowMobilityScore *
            SPEED_AUTOTUNE_LOW_MOBILITY_INTEGRAL_RPM;
        float unsaturatedA = proportionalA +
                             s_speedAutoTune.kpProbeIntegralA;
        /* 条件积分实现常规饱和抗积分饱和：只允许积分把输出拉回限幅区。 */
        bool canIntegrate =
            (unsaturatedA < g_speedAutoTuneResult.testCurrentA &&
             unsaturatedA > -g_speedAutoTuneResult.testCurrentA) ||
            (unsaturatedA >= g_speedAutoTuneResult.testCurrentA &&
             speedErrorElectricalHz < 0.0f) ||
            (unsaturatedA <= -g_speedAutoTuneResult.testCurrentA &&
             speedErrorElectricalHz > 0.0f);

        /*
         * 低于连续运动速度时，积分只会储存克服静摩擦所需的转矩，并在脱离静摩擦时
         * 释放为较大的起步脉冲。因此在轴尚未沿目标方向运动前保持积分为零，由 P 项
         * 负责越过静摩擦门限。
         */
        if (directedRpm < integralEnableRpm)
        {
            s_speedAutoTune.kpProbeIntegralA = 0.0f;
        }
        else if (canIntegrate)
        {
            s_speedAutoTune.kpProbeIntegralA +=
                probeKi * speedErrorElectricalHz /
                (float)PWM_FREQUENCY;
            s_speedAutoTune.kpProbeIntegralA = SpeedAutoTune_Clamp(
                s_speedAutoTune.kpProbeIntegralA,
                -integralLimitA,
                integralLimitA);
        }

        return SpeedAutoTune_Clamp(
            proportionalA + s_speedAutoTune.kpProbeIntegralA,
            -g_speedAutoTuneResult.testCurrentA,
            g_speedAutoTuneResult.testCurrentA);
    }

    return SpeedAutoTune_Clamp(proportionalA,
                               -g_speedAutoTuneResult.testCurrentA,
                               g_speedAutoTuneResult.testCurrentA);
}

static float SpeedAutoTune_GetKpProbeBrakeIq(void)
{
    uint32_t rampTicks = SPEED_AUTOTUNE_MS_TO_TICKS(
        SpeedAutoTune_GetKpRampMs());
    float progress = (rampTicks > 0U) ?
        (float)s_speedAutoTune.stageTicks / (float)rampTicks : 1.0f;
    float targetRpm = s_speedAutoTune.kpBrakeStartRpm *
        (1.0f - SpeedAutoTune_SmoothStep(progress));
    float speedErrorElectricalHz =
        (targetRpm - SpeedAutoTune_GetKpFeedbackRpm()) *
        (float)POLE_PAIR_NUM / 60.0f;
    float currentLimitA = SpeedAutoTune_BlendProfile(
        g_speedAutoTuneResult.testCurrentA,
        s_speedAutoTune.sineCurrentA);
    float commandA = s_speedAutoTune.kpProbeGain *
                     speedErrorElectricalHz;
    int8_t direction = (commandA > 0.0f) ?
        1 : ((commandA < 0.0f) ? -1 : 0);

    if (direction != 0 &&
        s_speedAutoTune.kpBrakeDirection != 0 &&
        direction != s_speedAutoTune.kpBrakeDirection)
    {
        PIDREGDQX_CURRENT_setUiQ_pu(
            &g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
    }
    if (direction != 0)
    {
        s_speedAutoTune.kpBrakeDirection = direction;
    }

    /* 停车稳定计时期间继续保持比例阻尼，防止转子在窗口边缘重新加速。 */
    return SpeedAutoTune_Clamp(commandA,
                               -currentLimitA,
                               currentLimitA);
}

static float SpeedAutoTune_GetCommandIq(void)
{
    switch (g_speedAutoTuneResult.state)
    {
        case SPEED_AUTOTUNE_SINE_ADAPT:
        case SPEED_AUTOTUNE_SINE_SETTLE:
        case SPEED_AUTOTUNE_SINE_MEASURE:
        case SPEED_AUTOTUNE_SINE_BRAKE:
            return s_speedAutoTune.sineCommandA;

        case SPEED_AUTOTUNE_KP_RAMP:
        case SPEED_AUTOTUNE_KP_PROBE:
            return SpeedAutoTune_GetKpProbeIq();

        case SPEED_AUTOTUNE_KP_PROBE_BRAKE:
            return SpeedAutoTune_GetKpProbeBrakeIq();

        default:
            return 0.0f;
    }
}

/*
 * 把三音测量统计转换为速度 PI 和 Kp 验证初值，依次执行：
 *
 * 1. 计算去直流总方差，为 Iq/速度相干性提供分母。
 * 2. 对每个音调恢复指令 Iq、实测 Iq、速度的幅值和相位，筛除弱响应或异常相位点。
 * 3. 按 (Iq/omega)^2 合并有效音调的加速度增益，并检查跟踪、相干性、速度响应、
 *    有效音调数量和削波比例。
 * 4. 由电流环 Wi 推导受限的速度环自然频率，再按二阶极点配置计算模型 Kp/Ki。
 * 5. 由电流安全上限确定 Kp 搜索上限，根据 lowMobilityScore 在模型种子和摩擦种子
 *    之间插值，初始化后续双向低速验证。
 *
 * 任一步质量检查失败都会调用 SpeedAutoTune_Fail() 并返回 false；返回 true 只表示
 * 模型计算完成，最终结果仍保持 valid=false，必须等待 Kp 实测验证。
 */
static bool SpeedAutoTune_CalculateResult(void)
{
    float inverseSamples;
    float commandAmplitudeA = 0.0f;
    float iqAmplitudeA = 0.0f;
    float speedAmplitudeElectricalHz = 0.0f;
    float speedAmplitudeRpm;
    uint32_t validToneCount = 0U;
    float accelerationWeightedSum = 0.0f;
    float accelerationWeightSum = 0.0f;
    float baseToneTrackingRatio = 0.0f;
    float trackingRatioForGate;
    float commandAmplitudeSquareSum = 0.0f;
    float iqAmplitudeSquareSum = 0.0f;
    float speedAmplitudeSquareSum = 0.0f;
    float iqMean;
    float speedMean;
    float iqVariance;
    float speedVariance;
    float iqCoherence;
    float speedCoherence;
    float currentTrackingRatio;
    float minimumTrackingRatio;
    float clippedFraction;
    float responsePhaseDeg = 0.0f;
    float minimumResponseRpm;
    float accelerationGain;
    float currentWiRadPerS;
    float targetNaturalFrequencyHz;
    float targetOmega;
    float modelKp;
    float modelKi;
    float probeSeedKp;

    /*
     * 结果计算第 1 步：计算去直流后的 Iq 和速度总方差，供后续相干性门限使用。
     * 方差下限避免纯零输入或浮点舍入造成除零。
     */
    if (s_speedAutoTune.sineSamples == 0U)
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_INVALID_RESPONSE);
        return false;
    }

    inverseSamples = 1.0f / (float)s_speedAutoTune.sineSamples;
    iqMean = s_speedAutoTune.iqSum * inverseSamples;
    speedMean = s_speedAutoTune.speedSum * inverseSamples;
    iqVariance = s_speedAutoTune.iqSquareSum * inverseSamples -
                 iqMean * iqMean;
    speedVariance = s_speedAutoTune.speedSquareSum * inverseSamples -
                    speedMean * speedMean;
    iqVariance = fmaxf(iqVariance, 1.0e-12f);
    speedVariance = fmaxf(speedVariance, 1.0e-12f);

    memset(g_speedAutoTuneResult.frfFrequencyHz, 0,
           sizeof(g_speedAutoTuneResult.frfFrequencyHz));
    memset(g_speedAutoTuneResult.frfMeasuredIqAmplitudeA, 0,
           sizeof(g_speedAutoTuneResult.frfMeasuredIqAmplitudeA));
    memset(g_speedAutoTuneResult.frfSpeedAmplitudeRpm, 0,
           sizeof(g_speedAutoTuneResult.frfSpeedAmplitudeRpm));
    memset(g_speedAutoTuneResult.frfPhaseDeg, 0,
           sizeof(g_speedAutoTuneResult.frfPhaseDeg));
    memset(g_speedAutoTuneResult.frfAccelerationGain, 0,
           sizeof(g_speedAutoTuneResult.frfAccelerationGain));

    /*
     * 结果计算第 2 步：重建每个音调的实测 Iq 和速度复数相量。由 Xcos、Xsin 两个
     * 投影通过 hypot(Xcos, Xsin) 得到幅值；速度相量除以电流相量得到响应相位；再按
     * 惯性关系 b = omega*|speed|/|Iq| 得到该音调的加速度增益估计。
     */
    for (uint32_t tone = 0U;
         tone < SPEED_AUTOTUNE_MULTISINE_TONE_COUNT;
         tone++)
    {
        float iqSin = 2.0f * s_speedAutoTune.iqSinSum[tone] *
                      inverseSamples;
        float iqCos = 2.0f * s_speedAutoTune.iqCosSum[tone] *
                      inverseSamples;
        float commandSin = 2.0f *
            s_speedAutoTune.commandSinSum[tone] * inverseSamples;
        float commandCos = 2.0f *
            s_speedAutoTune.commandCosSum[tone] * inverseSamples;
        float speedSin = 2.0f *
            s_speedAutoTune.speedSinSum[tone] * inverseSamples;
        float speedCos = 2.0f *
            s_speedAutoTune.speedCosSum[tone] * inverseSamples;
        float commandToneAmplitudeA = sqrtf(
            commandSin * commandSin + commandCos * commandCos);
        float iqToneAmplitudeA = sqrtf(
            iqSin * iqSin + iqCos * iqCos);
        float speedToneAmplitudeElectricalHz = sqrtf(
            speedSin * speedSin + speedCos * speedCos);
        float speedToneAmplitudeRpm =
            speedToneAmplitudeElectricalHz * 60.0f /
            (float)POLE_PAIR_NUM;
        float toneTrackingRatio = (commandToneAmplitudeA > 1.0e-9f) ?
            iqToneAmplitudeA / commandToneAmplitudeA : 0.0f;
        float transferDenominator = iqCos * iqCos + iqSin * iqSin;
        float phaseDeg = 0.0f;
        float toneAccelerationGain = 0.0f;

        if (transferDenominator > 1.0e-12f)
        {
            float transferReal =
                (speedCos * iqCos + speedSin * iqSin) /
                transferDenominator;
            float transferImaginary =
                (speedCos * iqSin - speedSin * iqCos) /
                transferDenominator;
            phaseDeg = atan2f(transferImaginary, transferReal) *
                       SPEED_AUTOTUNE_RAD_TO_DEG;
        }

        if (iqToneAmplitudeA > 1.0e-9f)
        {
            toneAccelerationGain = 2.0f * SPEED_AUTOTUNE_PI *
                s_multisineFrequencyHz[tone] *
                speedToneAmplitudeElectricalHz / iqToneAmplitudeA;
        }

        commandAmplitudeSquareSum +=
            commandToneAmplitudeA * commandToneAmplitudeA;
        iqAmplitudeSquareSum +=
            iqToneAmplitudeA * iqToneAmplitudeA;
        speedAmplitudeSquareSum +=
            speedToneAmplitudeElectricalHz *
            speedToneAmplitudeElectricalHz;

        g_speedAutoTuneResult.frfFrequencyHz[tone] =
            s_multisineFrequencyHz[tone];
        g_speedAutoTuneResult.frfMeasuredIqAmplitudeA[tone] =
            iqToneAmplitudeA;
        g_speedAutoTuneResult.frfSpeedAmplitudeRpm[tone] =
            speedToneAmplitudeRpm;
        g_speedAutoTuneResult.frfPhaseDeg[tone] = phaseDeg;
        g_speedAutoTuneResult.frfAccelerationGain[tone] =
            toneAccelerationGain;

        if (tone == 0U)
        {
            baseToneTrackingRatio = toneTrackingRatio;
        }

        if (isfinite(toneAccelerationGain) &&
            toneAccelerationGain > 0.0f &&
            iqToneAmplitudeA >=
                SPEED_AUTOTUNE_MULTISINE_MIN_IQ_AMPLITUDE_A &&
            speedToneAmplitudeRpm >=
                SPEED_AUTOTUNE_MULTISINE_MIN_SPEED_RPM &&
            toneTrackingRatio >=
                SPEED_AUTOTUNE_MULTISINE_MIN_TRACKING_RATIO &&
            toneTrackingRatio <=
                SPEED_AUTOTUNE_MAX_IQ_TRACKING_RATIO &&
            phaseDeg >= SPEED_AUTOTUNE_MULTISINE_MIN_PHASE_DEG &&
            phaseDeg <= SPEED_AUTOTUNE_MULTISINE_MAX_PHASE_DEG)
        {
            float omega = 2.0f * SPEED_AUTOTUNE_PI *
                          s_multisineFrequencyHz[tone];
            /*
             * 使用 (Iq/omega)^2 加权。对惯性响应，该权重与速度幅值平方成比例，可防止
             * 响应微弱、噪声较大的高频点主导组合后的模型增益。
             */
            float fitWeight = iqToneAmplitudeA / omega;

            fitWeight *= fitWeight;
            accelerationWeightedSum +=
                toneAccelerationGain * fitWeight;
            accelerationWeightSum += fitWeight;
            validToneCount++;
        }

        printf("[SPEED_AUTOTUNE] FRF[%lu]: f=%.3f Hz, commandIq=%.4f A, "
               "measuredIq=%.4f A, tracking=%.3f, speedAmp=%.4f rpm, "
               "phase=%.1f deg, accelGain=%.3f\n",
               (unsigned long)tone,
               s_multisineFrequencyHz[tone],
               commandToneAmplitudeA,
               iqToneAmplitudeA,
               toneTrackingRatio,
               speedToneAmplitudeRpm,
               phaseDeg,
               toneAccelerationGain);
    }

    /*
     * 结果计算第 3 步：合并相互正交的音调能量，并拒绝不足以支持可信模型的数据。
     * 相干性比较“注入音调解释的正弦方差 sum(A_tone^2)/2”和“去直流时域总方差”。
     */
    commandAmplitudeA = sqrtf(commandAmplitudeSquareSum);
    iqAmplitudeA = sqrtf(iqAmplitudeSquareSum);
    speedAmplitudeElectricalHz = sqrtf(speedAmplitudeSquareSum);
    speedAmplitudeRpm = speedAmplitudeElectricalHz *
                        60.0f / (float)POLE_PAIR_NUM;
    iqCoherence = SpeedAutoTune_Clamp(
        0.5f * iqAmplitudeSquareSum / iqVariance,
        0.0f, 1.0f);
    speedCoherence = SpeedAutoTune_Clamp(
        0.5f * speedAmplitudeSquareSum / speedVariance,
        0.0f, 1.0f);
    currentTrackingRatio = (commandAmplitudeA > 1.0e-9f) ?
                           iqAmplitudeA / commandAmplitudeA : 0.0f;
    clippedFraction = (float)s_speedAutoTune.sineClippedSamples *
                      inverseSamples;

    responsePhaseDeg = g_speedAutoTuneResult.frfPhaseDeg[0U];

    g_speedAutoTuneResult.sampleCount = s_speedAutoTune.sineSamples;
    g_speedAutoTuneResult.measuredCycles =
        SPEED_AUTOTUNE_SINE_MEASURE_CYCLES;
    g_speedAutoTuneResult.excitationCurrentA =
        s_speedAutoTune.sineCurrentA;
    g_speedAutoTuneResult.excitationFrequencyHz =
        SPEED_AUTOTUNE_SINE_FREQUENCY_HZ;
    g_speedAutoTuneResult.commandedIqAmplitudeA = commandAmplitudeA;
    g_speedAutoTuneResult.measuredIqAmplitudeA = iqAmplitudeA;
    g_speedAutoTuneResult.speedAmplitudeRpm = speedAmplitudeRpm;
    g_speedAutoTuneResult.currentTrackingRatio = currentTrackingRatio;
    g_speedAutoTuneResult.iqCoherence = iqCoherence;
    g_speedAutoTuneResult.speedCoherence = speedCoherence;
    g_speedAutoTuneResult.responsePhaseDeg = responsePhaseDeg;
    g_speedAutoTuneResult.clippedFraction = clippedFraction;
    g_speedAutoTuneResult.validFrfToneCount = validToneCount;

    minimumResponseRpm = SpeedAutoTune_BlendProfile(
        SPEED_AUTOTUNE_SINE_ADAPT_TARGET_RPM *
            SPEED_AUTOTUNE_MULTISINE_PER_TONE_RATIO *
            SPEED_AUTOTUNE_MULTISINE_RESPONSE_RETENTION,
        SPEED_AUTOTUNE_CEILING_MIN_RESPONSE_RPM *
            SPEED_AUTOTUNE_MULTISINE_PER_TONE_RATIO);
    minimumTrackingRatio = SpeedAutoTune_BlendProfile(
        SPEED_AUTOTUNE_MIN_IQ_TRACKING_RATIO,
        SPEED_AUTOTUNE_CEILING_MIN_IQ_TRACKING_RATIO);
    trackingRatioForGate = SpeedAutoTune_BlendProfile(
        currentTrackingRatio, baseToneTrackingRatio);

    printf("[SPEED_AUTOTUNE] multisine response: tones=%lu, base=%.3f Hz, "
           "forcingIq=%.4f A, "
           "commandIq=%.4f A, measuredIq=%.4f A, tracking=%.3f, "
           "speedAmp=%.3f rpm\n",
           (unsigned long)validToneCount,
           SPEED_AUTOTUNE_SINE_FREQUENCY_HZ,
           s_speedAutoTune.sineCurrentA,
           commandAmplitudeA,
           iqAmplitudeA,
           currentTrackingRatio,
           speedAmplitudeRpm);
    printf("[SPEED_AUTOTUNE] multisine quality: iqCoherence=%.3f, "
           "speedCoherence=%.3f, baseTracking=%.3f, phase=%.1f deg, "
           "clipped=%.3f, requiredSpeed=%.3f rpm, ceilingFallback=%u\n",
           iqCoherence,
           speedCoherence,
           baseToneTrackingRatio,
           responsePhaseDeg,
           clippedFraction,
           minimumResponseRpm,
           s_speedAutoTune.sineCeilingResponseAccepted ? 1U : 0U);

    if (!isfinite(iqAmplitudeA) || !isfinite(trackingRatioForGate) ||
        trackingRatioForGate < minimumTrackingRatio ||
        trackingRatioForGate > SPEED_AUTOTUNE_MAX_IQ_TRACKING_RATIO ||
        iqCoherence < SPEED_AUTOTUNE_MIN_IQ_COHERENCE ||
        clippedFraction > SPEED_AUTOTUNE_MAX_CLIPPED_FRACTION)
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_CURRENT_TRACKING);
        return false;
    }

    if (!isfinite(speedAmplitudeRpm) ||
        speedAmplitudeRpm < minimumResponseRpm ||
        speedCoherence < SPEED_AUTOTUNE_MIN_SPEED_COHERENCE ||
        validToneCount < SPEED_AUTOTUNE_MULTISINE_MIN_VALID_TONES)
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_INVALID_RESPONSE);
        return false;
    }

    if (!isfinite(accelerationWeightSum) ||
        accelerationWeightSum <= 1.0e-12f)
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_INVALID_RESPONSE);
        return false;
    }
    accelerationGain = accelerationWeightedSum /
                       accelerationWeightSum;

    /*
     * 结果计算第 4 步：根据已经整定的电流环带宽选择速度环带宽。速度环至少比电流环
     * 慢 CURRENT_TO_SPEED_RATIO 倍，然后再限制到配置的实际可用范围。
     */
    currentWiRadPerS = fabsf(
        PIDREGDQX_CURRENT_getWi_si(&g_axis.currCtrl.pidIdIq));
    targetNaturalFrequencyHz = currentWiRadPerS /
        (2.0f * SPEED_AUTOTUNE_PI *
         SPEED_AUTOTUNE_CURRENT_TO_SPEED_RATIO);
    targetNaturalFrequencyHz = SpeedAutoTune_Clamp(
        targetNaturalFrequencyHz,
        SPEED_AUTOTUNE_MIN_NATURAL_FREQ_HZ,
        SPEED_AUTOTUNE_MAX_NATURAL_FREQ_HZ);
    targetOmega = 2.0f * SPEED_AUTOTUNE_PI *
                  targetNaturalFrequencyHz;

    /*
     * 对对象 P(s)=b/s 和 PI 控制器 C(s)=Kp+Ki/s，闭环特征多项式为：
     *
     *   s^2 + b*Kp*s + b*Ki.
     *
     * 与目标二阶多项式 s^2 + 2*zeta*omega_n*s + omega_n^2 对应，可得到下面的
     * Kp/Ki 公式。单位分别为 A/eHz 和 A/(eHz*s)。
     */
    modelKp = 2.0f * SPEED_AUTOTUNE_TARGET_DAMPING_RATIO *
              targetOmega / accelerationGain;
    modelKi = targetOmega * targetOmega / accelerationGain;

    if (!isfinite(accelerationGain) || !isfinite(modelKp) ||
        !isfinite(modelKi) || accelerationGain <= 0.0f ||
        modelKp <= 0.0f || modelKp > 20.0f ||
        modelKi <= 0.0f || modelKi > 2000.0f)
    {
        printf("[SPEED_AUTOTUNE] rejected model: accelGain=%.3f, "
               "Kp=%.6f, Ki=%.6f\n",
               accelerationGain, modelKp, modelKi);
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_INVALID_GAIN);
        return false;
    }

    g_speedAutoTuneResult.accelerationGain_eHz_per_s_per_A =
        accelerationGain;
    g_speedAutoTuneResult.targetNaturalFrequencyHz =
        targetNaturalFrequencyHz;
    g_speedAutoTuneResult.targetDampingRatio =
        SPEED_AUTOTUNE_TARGET_DAMPING_RATIO;
    g_speedAutoTuneResult.modelKp_A_per_eHz = modelKp;
    g_speedAutoTuneResult.modelKi_A_per_eHz_s = modelKi;
    g_speedAutoTuneResult.probedKp_A_per_eHz = 0.0f;
    g_speedAutoTuneResult.kp_A_per_eHz = modelKp;
    g_speedAutoTuneResult.ki_A_per_eHz_s = modelKi;
    g_speedAutoTuneResult.valid = false;

    /*
     * 结果计算第 5 步：准备有界的实测 Kp 验证。Kp_max 定义为固定 2 rpm 误差下刚好
     * 请求到调用方授权电流上限的增益，因此后续几何递增搜索不会突破电流约束。
     */
    s_speedAutoTune.kpProbeMaximumGain =
        g_speedAutoTuneResult.testCurrentA /
        (SPEED_AUTOTUNE_KP_PROBE_FULL_CURRENT_ERROR_RPM *
         (float)POLE_PAIR_NUM / 60.0f);

    /*
     * 在线性模型种子 Kp 与克服摩擦所需的种子 Kp 之间插值。插值权重由实测对象机动性
     * 决定，但所有对象仍共用同一个电流/误差安全上限。
     */
    {
        float targetElectricalHz =
            SpeedAutoTune_GetKpValidationTargetRpm() *
            (float)POLE_PAIR_NUM / 60.0f;
        float modelSeedKp = fminf(
            modelKp, s_speedAutoTune.kpProbeMaximumGain);
        float frictionSeedKp =
            SPEED_AUTOTUNE_LOW_MOBILITY_SEED_I_RATIO *
            s_speedAutoTune.sineCurrentA /
            fmaxf(targetElectricalHz, 1.0e-6f);

        probeSeedKp = SpeedAutoTune_BlendProfile(
            modelSeedKp, frictionSeedKp);
        s_speedAutoTune.kpProbeGain = SpeedAutoTune_Clamp(
            probeSeedKp, 1.0e-6f,
            s_speedAutoTune.kpProbeMaximumGain);
    }
    s_speedAutoTune.kpProbeAttempts = 1U;
    s_speedAutoTune.kpProbePasses = 0U;
    s_speedAutoTune.kpProbeIntegralA = 0.0f;
    s_speedAutoTune.kpProbeTargetRpm =
        SpeedAutoTune_GetKpValidationTargetRpm();
    s_speedAutoTune.kpRampStartRpm = 0.0f;
    s_speedAutoTune.kpRampEndRpm =
        SpeedAutoTune_GetKpValidationTargetRpm();
    s_speedAutoTune.kpWindowCount = 0U;
    s_speedAutoTune.kpBestValid = false;
    s_speedAutoTune.kpBestStrict = false;
    s_speedAutoTune.kpBestUtility = -INFINITY;
    s_speedAutoTune.kpWorseCandidates = 0U;
    SpeedAutoTune_ResetKpCandidate();
    SpeedAutoTune_ResetKpProbeMeasurement();
    return true;
}

void SpeedAutoTune_Start(float testCurrentA)
{
    float selectedCeiling = testCurrentA;

    if (!isfinite(selectedCeiling) || selectedCeiling <= 0.0f)
    {
        selectedCeiling = SPEED_AUTOTUNE_DEFAULT_CURRENT_A;
    }
    selectedCeiling = SpeedAutoTune_Clamp(
        selectedCeiling,
        SPEED_AUTOTUNE_MIN_CURRENT_A,
        SPEED_AUTOTUNE_MAX_CURRENT_A);

    /*
     * Start() 只准备 RAM 状态。任务/CLI 调用方会另外置位 g_bStartSpeedAutoTune 并
     * 切换轴模式，第一次 Handle() 调用才执行从 IDLE 到 SETTLE 的硬件安全过渡。
     * 此处清空公共结果会立即令 valid=false，避免调用方把上一次结果误认为本次结果。
     */
    memset(&s_speedAutoTune, 0, sizeof(s_speedAutoTune));
    memset(&g_speedAutoTuneResult, 0, sizeof(g_speedAutoTuneResult));
    g_speedAutoTuneResult.state = SPEED_AUTOTUNE_IDLE;
    g_speedAutoTuneResult.error = SPEED_AUTOTUNE_ERROR_NONE;
    g_speedAutoTuneResult.testCurrentA = selectedCeiling;
    s_speedAutoTune.sineCurrentA = SpeedAutoTune_Clamp(
        SPEED_AUTOTUNE_SINE_INITIAL_CURRENT_A,
        SPEED_AUTOTUNE_SINE_INITIAL_CURRENT_A,
        selectedCeiling);
    s_speedAutoTune.oldKp =
        PIDREG_SPEED_getKp_si(&g_axis.speedCtrl.PIDSpeed);
    s_speedAutoTune.oldKi =
        PIDREG_SPEED_getKi_si(&g_axis.speedCtrl.PIDSpeed);
    s_speedAutoTune.sineAdaptCycles = 0U;
    s_speedAutoTune.sineAdaptDwellCycles = 0U;
    SpeedAutoTune_ResetSineAdaptCycle();

    printf("[SPEED_AUTOTUNE] prepared adaptive sine + 3-tone FRF: "
           "base=%.3f Hz, "
           "IqInitial=%.3f A, IqCeiling=%.3f A, "
           "adaptTarget=%.1f rpm, centerScale=%.1f rpm\n",
           SPEED_AUTOTUNE_SINE_FREQUENCY_HZ,
           s_speedAutoTune.sineCurrentA,
           selectedCeiling,
           SPEED_AUTOTUNE_SINE_ADAPT_TARGET_RPM,
           SPEED_AUTOTUNE_SINE_TARGET_AMPLITUDE_RPM);
    printf("[SPEED_AUTOTUNE] prepared mobility-profiled Kp validation: "
           "target=%.2f..%.2f rpm, ramp=%u..%u ms, "
           "window=%u+%u..%u+%u ms, "
           "requiredPasses=%u\n",
           SPEED_AUTOTUNE_KP_PROBE_TARGET_RPM,
           SPEED_AUTOTUNE_LOW_MOBILITY_KP_TARGET_RPM,
           (unsigned int)SPEED_AUTOTUNE_KP_PROBE_RAMP_MS,
           (unsigned int)SPEED_AUTOTUNE_LOW_MOBILITY_KP_RAMP_MS,
           (unsigned int)SPEED_AUTOTUNE_KP_PROBE_SETTLE_MS,
           (unsigned int)SPEED_AUTOTUNE_KP_PROBE_MEASURE_MS,
           (unsigned int)SPEED_AUTOTUNE_LOW_MOBILITY_KP_SETTLE_MS,
           (unsigned int)SPEED_AUTOTUNE_LOW_MOBILITY_KP_MEASURE_MS,
           (unsigned int)SPEED_AUTOTUNE_KP_PROBE_REQUIRED_PASSES);
}

void SpeedAutoTune_Abort(void)
{
    bool wasActive = g_bStartSpeedAutoTune ||
                     g_axis.state == AXIS_STATE_SPEED_AUTOTUNE;

    if (!wasActive)
    {
        return;
    }

    SwitchOff_PWM(g_axis.pPWMCHandle);
    MC_Reset_Control_State();
    g_bStartSpeedAutoTune = false;
    g_speedAutoTuneResult.valid = false;
    g_speedAutoTuneResult.state = SPEED_AUTOTUNE_IDLE;
    g_speedAutoTuneResult.error = SPEED_AUTOTUNE_ERROR_NONE;
    printf("[SPEED_AUTOTUNE] aborted\n");
}

static bool SpeedAutoTune_RecoverFoc(CurrCtrlInput_t *currentInput)
{

    /*
     * 开环实验结束后保持 PWM 关闭若干个高频调用，使定时器和电流控制状态稳定。重新
     * 开启 PWM 前先执行一次零电流 FOC，避免进入闭环验证或恢复正常 RUN 时输出旧占空比。
     */
    SwitchOff_PWM(g_axis.pPWMCHandle);
    LL_TIM_ClearFlag_UPDATE(TIM1);

    s_speedAutoTune.recoveryHfTicks++;
    if (s_speedAutoTune.recoveryHfTicks <
        SPEED_AUTOTUNE_RECOVERY_HF_TICKS)
    {
        return false;
    }

    Get_RST_Measurements(g_axis.pPWMCHandle,
                         &g_axis.currCtrl.IrstMeas,
                         &g_axis.VotlMeas.VrstMeas);
    g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
    g_axis.currCtrl.refIdq.Q = FIXP30(0.0f);
    currentInput->Irst_in_pu = g_axis.currCtrl.IrstMeas;
    currentInput->Udcbus_in_pu = g_axis.busVoltage;

    LL_TIM_ClearFlag_UPDATE(TIM1);
    Curr_Control(&g_axis.currCtrl, currentInput);

    LL_TIM_ClearFlag_UPDATE(TIM1);
    SwitchOn_PWM(g_axis.pPWMCHandle);
    return true;
}

/*
 * 速度辨识高频入口，必须按 PWM_FREQUENCY 周期调用。执行顺序为：
 *
 * 1. 优先处理要求 PWM 关闭的 SOLVE、两类 FOC 恢复、FINISH 和 FAULT 状态。
 * 2. 活跃实验状态下读取相电流并执行独立过流检查。
 * 3. IDLE 首次进入时关闭 PWM、清控制状态、建立编码器展开基准并转入 SETTLE。
 * 4. 根据当前状态生成 Iq 指令，直接运行一次电流 FOC，再开启 PWM 输出本拍占空比。
 * 5. 每 SPEED_CONTROL_COUNT 次高频调用更新一次速度、位移/超速保护和状态机。
 *
 * 该函数绕过正常速度控制器直接控制 Iq，所以任何提前返回都必须明确保持 PWM 状态；
 * 故障统一由 SpeedAutoTune_Fail() 关闭 PWM 并使结果失效。
 */
void SpeedAutoTune_Handle(void)
{
    CurrCtrlInput_t currentInput;
    float iqCommandA;

    /*
     * SOLVE 和恢复状态必须在正常 PWM 频率路径之前处理，因为这些阶段有意要求 PWM
     * 关闭。处理后立即返回，可防止下面的通用指令/电流控制代码重新开启 PWM。
     */
    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_SOLVE)
    {
        SwitchOff_PWM(g_axis.pPWMCHandle);
        SpeedAutoTune_PrintStageHistory();
        if (SpeedAutoTune_CalculateResult())
        {
            float probeKiScale =
                g_speedAutoTuneResult.lowMobilityScore *
                SPEED_AUTOTUNE_LOW_MOBILITY_KI_SCALE;
            float probeKi = g_speedAutoTuneResult.modelKi_A_per_eHz_s *
                s_speedAutoTune.kpProbeGain /
                g_speedAutoTuneResult.modelKp_A_per_eHz *
                probeKiScale;

            printf("[SPEED_AUTOTUNE] starting smooth speed PI validation: "
                   "modelKp=%.6f, seedKp=%.6f, seedKi=%.6f, "
                   "maxKp=%.6f, "
                   "target=%.2f rpm, ramp=%lu ms, window=%lu+%lu ms, "
                   "integralEnable=%.2f rpm, kiScale=%.2f, "
                   "step=%.2f, feedback=kalman, profile=%s, score=%.3f\n",
                   g_speedAutoTuneResult.modelKp_A_per_eHz,
                   s_speedAutoTune.kpProbeGain,
                   probeKi,
                   s_speedAutoTune.kpProbeMaximumGain,
                   SpeedAutoTune_GetKpValidationTargetRpm(),
                   (unsigned long)SpeedAutoTune_GetKpRampMs(),
                   (unsigned long)SpeedAutoTune_GetKpSettleMs(),
                   (unsigned long)SpeedAutoTune_GetKpMeasureMs(),
                   g_speedAutoTuneResult.lowMobilityScore *
                       SPEED_AUTOTUNE_LOW_MOBILITY_INTEGRAL_RPM,
                   probeKiScale,
                   SpeedAutoTune_GetKpStepRatio(),
                   SpeedAutoTune_GetPlantProfileName(),
                   g_speedAutoTuneResult.lowMobilityScore);
            s_speedAutoTune.recoveryHfTicks = 0U;
            LL_TIM_ClearFlag_UPDATE(TIM1);
            SpeedAutoTune_EnterState(SPEED_AUTOTUNE_KP_PROBE_RECOVER);
        }
        return;
    }

    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_KP_PROBE_RECOVER)
    {
        if (SpeedAutoTune_RecoverFoc(&currentInput))
        {
            SpeedAutoTune_EnterState(SPEED_AUTOTUNE_KP_RAMP);
        }
        return;
    }

    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FINISH)
    {
        SwitchOff_PWM(g_axis.pPWMCHandle);
        MC_Reset_Control_State();
        MC_Set_Control_Mode(CTRL_MODE_SPEED);

        printf("[SPEED_AUTOTUNE] multisine result: samples=%lu, "
               "baseCycles=%lu, base=%.3f Hz, forcingIq=%.4f A, "
               "commandIq=%.4f A, "
               "measuredIq=%.4f A, "
               "speedAmp=%.3f rpm\n",
               (unsigned long)g_speedAutoTuneResult.sampleCount,
               (unsigned long)g_speedAutoTuneResult.measuredCycles,
               g_speedAutoTuneResult.excitationFrequencyHz,
               g_speedAutoTuneResult.excitationCurrentA,
               g_speedAutoTuneResult.commandedIqAmplitudeA,
               g_speedAutoTuneResult.measuredIqAmplitudeA,
               g_speedAutoTuneResult.speedAmplitudeRpm);
        printf("[SPEED_AUTOTUNE] quality: tracking=%.3f, "
               "iqCoherence=%.3f, speedCoherence=%.3f, "
               "phase=%.1f deg, clipped=%.3f\n",
               g_speedAutoTuneResult.currentTrackingRatio,
               g_speedAutoTuneResult.iqCoherence,
               g_speedAutoTuneResult.speedCoherence,
               g_speedAutoTuneResult.responsePhaseDeg,
               g_speedAutoTuneResult.clippedFraction);
        printf("[SPEED_AUTOTUNE] identified accelGain=%.3f eHz/(s*A), "
               "naturalFreq=%.3f Hz, damping=%.2f, "
               "modelPI=%.6f/%.6f\n",
               g_speedAutoTuneResult.accelerationGain_eHz_per_s_per_A,
               g_speedAutoTuneResult.targetNaturalFrequencyHz,
               g_speedAutoTuneResult.targetDampingRatio,
               g_speedAutoTuneResult.modelKp_A_per_eHz,
               g_speedAutoTuneResult.modelKi_A_per_eHz_s);
//        for (uint32_t index = 0U;
//             index < s_speedAutoTune.kpWindowCount;
//             index++)
//        {
//            printf("[SPEED_AUTOTUNE] Kp window[%lu]: target=%+.1f rpm, "
//                   "Kp=%.6f, directedMean=%.3f rpm, "
//                   "range=%.3f..%.3f rpm, rmsError=%.3f rpm, pass=%u\n",
//                   (unsigned long)index,
//                   s_speedAutoTune.kpWindowTargetRpm[index],
//                   s_speedAutoTune.kpWindowGain[index],
//                   s_speedAutoTune.kpWindowMeanRpm[index],
//                   s_speedAutoTune.kpWindowMinimumRpm[index],
//                   s_speedAutoTune.kpWindowMaximumRpm[index],
//                   s_speedAutoTune.kpWindowRmsErrorRpm[index],
//                   s_speedAutoTune.kpWindowPassed[index] ? 1U : 0U);
//        }

        printf("[SPEED_AUTOTUNE] PI applied in RAM: Kp=%f A/eHz, "
               "Ki=%f A/(eHz*s), selection=%s, profile=%s, "
               "score=%.3f, utility=%.3f, lowMobilityScore=%.3f\n",
               g_speedAutoTuneResult.kp_A_per_eHz,
               g_speedAutoTuneResult.ki_A_per_eHz_s,
               SpeedAutoTune_GetSelectionName(),
               SpeedAutoTune_GetPlantProfileName(),
               g_speedAutoTuneResult.lowMobilityScore,
               s_speedAutoTune.kpBestUtility,
			   g_speedAutoTuneResult.lowMobilityScore);

        PIDREG_SPEED_setKp_si(
            &g_axis.speedCtrl.PIDSpeed,
            g_speedAutoTuneResult.kp_A_per_eHz);
        PIDREG_SPEED_setKi_si(
            &g_axis.speedCtrl.PIDSpeed,
            g_speedAutoTuneResult.ki_A_per_eHz_s);

        s_speedAutoTune.recoveryHfTicks = 0U;
        LL_TIM_ClearFlag_UPDATE(TIM1);
        SpeedAutoTune_EnterState(SPEED_AUTOTUNE_RECOVER);
        return;
    }

    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_RECOVER)
    {
        if (SpeedAutoTune_RecoverFoc(&currentInput))
        {
            g_bStartSpeedAutoTune = false;
            g_axis.state = AXIS_STATE_RUN;
        }
        return;
    }

    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FAULT)
    {
        SwitchOff_PWM(g_axis.pPWMCHandle);
        return;
    }

    /*
     * 有源实验路径按 PWM_FREQUENCY 执行。相电流检查独立于 dq 电流限幅，因此能在
     * 输出下一拍占空比前发现坐标变换或传感器异常。
     */
    Get_RST_Measurements(g_axis.pPWMCHandle,
                         &g_axis.currCtrl.IrstMeas,
                         &g_axis.VotlMeas.VrstMeas);
    if (SpeedAutoTune_GetPhaseCurrentPeakA() >
        SPEED_AUTOTUNE_MAX_PHASE_CURRENT_A)
    {
        SpeedAutoTune_Fail(SPEED_AUTOTUNE_ERROR_OVERCURRENT);
        return;
    }

    if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_IDLE)
    {
        SwitchOff_PWM(g_axis.pPWMCHandle);
        MC_Reset_Control_State();
        s_speedAutoTune.previousRaw = g_axis.fbdk.uAngleRaw;
        s_speedAutoTune.unwrappedCounts = 0;
        s_speedAutoTune.guardWindowStartCounts = 0;
        s_speedAutoTune.guardMechanicalRpm = 0.0f;
        s_speedAutoTune.sineCommandA = 0.0f;
        SpeedAutoTune_EnterState(SPEED_AUTOTUNE_SETTLE);
    }

    iqCommandA = SpeedAutoTune_GetCommandIq();
    g_axis.currCtrl.refIdq.D = FIXP30(0.0f);
    g_axis.currCtrl.refIdq.Q = FIXP30(iqCommandA / CURRENT_SCALE);

    currentInput.Irst_in_pu = g_axis.currCtrl.IrstMeas;
    currentInput.Udcbus_in_pu = g_axis.busVoltage;
    Curr_Control(&g_axis.currCtrl, &currentInput);
    SwitchOn_PWM(g_axis.pPWMCHandle);

    /*
     * 机械量估计和状态转换按 SPEED_CONTROL_RATE 执行；两次速度更新之间，当前指令
     * 会持续作用于每个电流环周期。因此激励相位和累计样本索引严格共享同一个低速率
     * 时间基准。
     */
    s_speedAutoTune.speedDivider++;
    if (s_speedAutoTune.speedDivider >= SPEED_CONTROL_COUNT)
    {
        s_speedAutoTune.speedDivider = 0U;
        Calc_Speed(&g_axis.speedCtrl.speedMeas_pu);
        SpeedAutoTune_UpdateKinematics();
        if (g_axis.state != AXIS_STATE_SPEED_AUTOTUNE)
        {
            return;
        }
        SpeedAutoTune_UpdateStage();
    }
}

const SpeedAutoTuneResult_t *SpeedAutoTune_GetResult(void)
{
    return &g_speedAutoTuneResult;
}
