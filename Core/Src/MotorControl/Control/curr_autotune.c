/*
 * curr_autotune.c
 *
 * 静止 PMSM 的 Rs/Ld/Lq 离线辨识。
 */

#include "curr_autotune.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "curr_fbdk.h"
#include "encoder.h"
#include "foc.h"
#include "main.h"
#include "mc_math.h"
#include "mc_interface.h"
#include "motor_control.h"
#include "speed_pos_fbdk.h"
#include "stm32g4xx_ll_tim.h"

RsIdent_t g_rs_ident;

typedef struct
{
    /* Rs 阶段的低速服务节拍和 L_ALIGN 阶段的对齐状态。 */
    uint16_t rsRateDivider;
    uint32_t stageTick;

    bool lockAngleValid;
    uint32_t lockRaw;
    uint32_t initialLockRaw;
    uint32_t lastRotorRaw;
    uint16_t encoderGlitchCount;
    uint32_t encoderNoiseRaw;
    uint32_t noiseSamples[AUTOTUNE_ENCODER_NOISE_SAMPLES];
    uint16_t noiseSampleCount;
    uint16_t noiseSampleIndex;
    uint32_t motionWindowStartRaw;
    uint16_t motionWindowTick;
    int8_t motionDirection;
    uint8_t movingWindowCount;
    uint8_t stationaryWindowCount;
    uint16_t rotorRebaseCount;
    float rsDuty;
    float baseDutyD;
    float alignCurrentSum;
    uint16_t alignCurrentCount;
    uint16_t alignStableCount;
    uint16_t alignEntryStableCount;
    bool alignInitialized;
    bool alignPolarityValid;
    bool alignProbeDecay;
    uint16_t alignProbeTick;
    uint16_t alignProbeSignCount;
    int8_t alignProbeSignCandidate;
    int8_t dqDutySign;

    /* 高频方波注入和单半周期积分状态。 */
    int8_t polarity;
    uint16_t halfTick;
    uint16_t completedCycles;
    bool segmentActive;
    float segmentStartCurrent;
    float segmentPrevCurrent;
    float segmentFlux;
    uint32_t segmentStartRaw;
    uint32_t segmentMaxDriftRaw;
    uint32_t maxObservedSegmentDriftRaw;
    uint16_t motionRejectedCount;
    float cycleMinCurrent;
    float cycleMaxCurrent;
    float lastCurrentPeakToPeak;
    float hfDuty;
    uint16_t adaptStableCount;

    /* Ld/Lq 样本及统计结果。每个有效半周期最多保存一个样本。 */
    float estimates[AUTOTUNE_MAX_ESTIMATES];
    uint16_t estimateCount;
    float positiveEstimateSum;
    float negativeEstimateSum;
    uint16_t positiveEstimateCount;
    uint16_t negativeEstimateCount;
    float lastMedian;
    float lastCoefficientOfVariation;
    uint16_t lastRetainedCount;

} CurrAutoTuneRuntime_t;

typedef enum
{
    AUTOTUNE_RS_REJECT_NONE = 0,
    AUTOTUNE_RS_REJECT_CURRENT_DELTA,
    AUTOTUNE_RS_REJECT_RESULT_RANGE
} AutoTuneRsRejectReason_t;

typedef enum
{
    AUTOTUNE_L_REJECT_NONE = 0,
    AUTOTUNE_L_REJECT_SAMPLE_COUNT,
    AUTOTUNE_L_REJECT_POLARITY_COUNT,
    AUTOTUNE_L_REJECT_POLARITY_MISMATCH,
    AUTOTUNE_L_REJECT_MEDIAN_RANGE,
    AUTOTUNE_L_REJECT_RETAINED_COUNT,
    AUTOTUNE_L_REJECT_RESULT_RANGE,
    AUTOTUNE_L_REJECT_SCATTER,
    AUTOTUNE_L_REJECT_ADAPT_RANGE
} AutoTuneLRejectReason_t;

static CurrAutoTuneRuntime_t s_runtime;
static AutoTuneRsRejectReason_t s_rsRejectReason;
static AutoTuneLRejectReason_t s_lRejectReason;

extern float s_prevEncoderRaw;

/* 把对外错误枚举转换成日志中的可读字符串。 */
static const char *AutoTune_ErrorName(CurrAutoTuneError_t error)
{
    switch (error)
    {
        case CURR_AUTOTUNE_ERROR_NONE: return "none";
        case CURR_AUTOTUNE_ERROR_RS_INVALID: return "Rs invalid";
        case CURR_AUTOTUNE_ERROR_BIAS_CURRENT: return "bias current invalid";
        case CURR_AUTOTUNE_ERROR_OVERCURRENT: return "injection overcurrent";
        case CURR_AUTOTUNE_ERROR_ROTOR_MOVED: return "rotor moved";
        case CURR_AUTOTUNE_ERROR_LD_INVALID: return "Ld invalid";
        case CURR_AUTOTUNE_ERROR_LQ_INVALID: return "Lq invalid";
        default: return "unknown";
    }
}

/* 把 Rs 计算阶段的内部拒绝原因转换成日志字符串。 */
static const char *AutoTune_RsRejectName(AutoTuneRsRejectReason_t reason)
{
    switch (reason)
    {
        case AUTOTUNE_RS_REJECT_NONE: return "none";
        case AUTOTUNE_RS_REJECT_CURRENT_DELTA: return "current delta too small";
        case AUTOTUNE_RS_REJECT_RESULT_RANGE: return "result out of range";
        default: return "unknown";
    }
}

/* 把 Ld/Lq 统计阶段的内部拒绝原因转换成日志字符串。 */
static const char *AutoTune_LRejectName(AutoTuneLRejectReason_t reason)
{
    switch (reason)
    {
        case AUTOTUNE_L_REJECT_NONE: return "none";
        case AUTOTUNE_L_REJECT_SAMPLE_COUNT: return "not enough total samples";
        case AUTOTUNE_L_REJECT_POLARITY_COUNT: return "not enough polarity samples";
        case AUTOTUNE_L_REJECT_POLARITY_MISMATCH: return "positive/negative mismatch";
        case AUTOTUNE_L_REJECT_MEDIAN_RANGE: return "median out of range";
        case AUTOTUNE_L_REJECT_RETAINED_COUNT: return "not enough retained samples";
        case AUTOTUNE_L_REJECT_RESULT_RANGE: return "mean result out of range";
        case AUTOTUNE_L_REJECT_SCATTER: return "sample scatter too large";
        case AUTOTUNE_L_REJECT_ADAPT_RANGE: return "injection current target not reached";
        default: return "unknown";
    }
}

/* 计算平均值的安全辅助函数，避免 count 为零时发生除零。 */
static float AutoTune_SafeMean(float sum, uint16_t count)
{
    return (count > 0U) ? (sum / (float)count) : 0.0f;
}

/* 将占空比或幅值限制在指定上下限内。 */
static float AutoTune_Clamp(float value, float minimum, float maximum)
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

static void AutoTune_ResetRunFeedback(void)
{
    g_axis.fbdk.fSpeedKalman = 0.0f;
    g_motorSpeedKalmanFilter.fOptimalEstimateValue = 0.0f;
    g_motorSpeedKalmanFilter.fErrorCovariance = 1.0f;
    s_prevEncoderRaw = (float)g_axis.fbdk.uAngleRaw;
    g_axis.speedCtrl.speedMeas_pu = FIXP30(0.0f);
}

/* 计算编码器环形计数的最短距离，处理从计数器末端回绕到零的情况。 */
static uint32_t AutoTune_RawAngleDifference(uint32_t a, uint32_t b)
{
    const uint32_t counts = (uint32_t)ENCODER_COUNT;
    uint32_t direct;
    uint32_t wrapped;

    if (counts == 0U)
    {
        return 0U;
    }

    a %= counts;
    b %= counts;
    direct = (a >= b) ? (a - b) : (b - a);
    wrapped = counts - direct;
    return (direct < wrapped) ? direct : wrapped;
}

/* Signed shortest distance on the encoder ring, used only for robust statistics. */
static int32_t AutoTune_RawAngleSignedDifference(uint32_t a, uint32_t b)
{
    const int32_t counts = (int32_t)ENCODER_COUNT;
    int32_t delta;

    if (counts <= 0)
    {
        return 0;
    }

    delta = (int32_t)(a % (uint32_t)counts) -
            (int32_t)(b % (uint32_t)counts);
    if (delta > (counts / 2))
    {
        delta -= counts;
    }
    else if (delta < -(counts / 2))
    {
        delta += counts;
    }
    return delta;
}

static uint32_t AutoTune_MaxU32(uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

/* Difference of two independent angle samples, plus one quantization count. */
static uint32_t AutoTune_EncoderDifferenceNoiseRaw(void)
{
    return 2U * AutoTune_MaxU32(s_runtime.encoderNoiseRaw, 1U) + 1U;
}

/* 将允许的电角度漂移换算成编码器原始计数。
 * 编码器记录的是机械角度，因此需要除以极对数。 */
static uint32_t AutoTune_RotorDriftRaw(float electricalDegrees)
{
    float limit = ENCODER_COUNT * electricalDegrees /
                  (360.0f * (float)POLE_PAIR_NUM);
    return (limit < 1.0f) ? 1U : (uint32_t)ceilf(limit);
}

static uint32_t AutoTune_MotionTrendLimitRaw(void)
{
    uint32_t physicalWindowLimit =
        (AutoTune_RotorDriftRaw(AUTOTUNE_SOFT_ELEC_DRIFT_DEG) +
         AUTOTUNE_MOTION_CONFIRM_WINDOWS - 1U) /
        AUTOTUNE_MOTION_CONFIRM_WINDOWS;
    return AutoTune_MaxU32(AutoTune_MaxU32(s_runtime.encoderNoiseRaw, 1U),
                           physicalWindowLimit);
}

static uint32_t AutoTune_SegmentDriftRaw(float electricalDegrees)
{
    uint32_t limit = AutoTune_RotorDriftRaw(electricalDegrees);
    return AutoTune_MaxU32(limit, AutoTune_EncoderDifferenceNoiseRaw());
}

static void AutoTune_ResetNoiseCalibration(void)
{
    s_runtime.noiseSampleCount = 0U;
    s_runtime.noiseSampleIndex = 0U;
}

/* Keep the most recent stable samples so alignment transients age out naturally. */
static void AutoTune_RecordStationaryAngle(void)
{
    s_runtime.noiseSamples[s_runtime.noiseSampleIndex] = g_axis.fbdk.uAngleRaw;
    s_runtime.noiseSampleIndex =
        (uint16_t)((s_runtime.noiseSampleIndex + 1U) % AUTOTUNE_ENCODER_NOISE_SAMPLES);
    if (s_runtime.noiseSampleCount < AUTOTUNE_ENCODER_NOISE_SAMPLES)
    {
        s_runtime.noiseSampleCount++;
    }
}

static void AutoTune_SortU32(uint32_t *values, uint16_t count)
{
    uint16_t i;

    for (i = 1U; i < count; i++)
    {
        uint32_t value = values[i];
        uint16_t j = i;
        while (j > 0U && values[j - 1U] > value)
        {
            values[j] = values[j - 1U];
            j--;
        }
        values[j] = value;
    }
}

/*
 * Estimate stationary encoder noise with median absolute deviation (MAD).
 * 1.4826 converts MAD to a Gaussian sigma; the configured sigma multiple then
 * gives a data-driven raw-count allowance without depending on encoder bits.
 */
static void AutoTune_UpdateEncoderNoise(const char *source)
{
    uint16_t count = s_runtime.noiseSampleCount;
    uint32_t reference;
    uint32_t median;
    uint32_t minimum;
    uint32_t maximum;
    uint32_t mad;
    uint32_t estimate;
    uint32_t maximumNoise = AutoTune_RotorDriftRaw(AUTOTUNE_NOISE_MAX_ELEC_DEG);
    bool contaminated;
    uint16_t i;

    if (count < (AUTOTUNE_ENCODER_NOISE_SAMPLES / 2U))
    {
        return;
    }

    reference = s_runtime.noiseSamples[0];
    for (i = 0U; i < count; i++)
    {
        int32_t offset = AutoTune_RawAngleSignedDifference(
            s_runtime.noiseSamples[i], reference);
        s_runtime.noiseSamples[i] =
            (uint32_t)(offset + (int32_t)ENCODER_COUNT);
    }
    AutoTune_SortU32(s_runtime.noiseSamples, count);
    minimum = s_runtime.noiseSamples[0];
    maximum = s_runtime.noiseSamples[count - 1U];
    median = s_runtime.noiseSamples[count / 2U];
    for (i = 0U; i < count; i++)
    {
        uint32_t value = s_runtime.noiseSamples[i];
        s_runtime.noiseSamples[i] =
            (value >= median) ? (value - median) : (median - value);
    }
    AutoTune_SortU32(s_runtime.noiseSamples, count);
    mad = s_runtime.noiseSamples[count / 2U];

    estimate = (uint32_t)ceilf(1.4826f * AUTOTUNE_ENCODER_NOISE_SIGMA *
                               (float)mad);
    estimate = AutoTune_MaxU32(estimate, 1U);
    contaminated = (uint32_t)(maximum - minimum) > (2U * maximumNoise);
    estimate = (estimate > maximumNoise) ? maximumNoise : estimate;
    s_runtime.encoderNoiseRaw =
        AutoTune_MaxU32(s_runtime.encoderNoiseRaw, estimate);
    printf("[AUTOTUNE] encoder noise (%s): MAD=%lu, span=%lu, "
           "allowance=%lu raw%s\n",
           source,
           (unsigned long)mad,
           (unsigned long)(maximum - minimum),
           (unsigned long)s_runtime.encoderNoiseRaw,
           contaminated ? " (span moving, MAD capped)" : "");
}

static void AutoTune_ResetMotionDetector(uint32_t raw)
{
    s_runtime.initialLockRaw = raw;
    s_runtime.lockRaw = raw;
    s_runtime.lastRotorRaw = raw;
    s_runtime.motionWindowStartRaw = raw;
    s_runtime.motionWindowTick = 0U;
    s_runtime.motionDirection = 0;
    s_runtime.movingWindowCount = 0U;
    s_runtime.stationaryWindowCount = 0U;
    s_runtime.rotorRebaseCount = 0U;
    s_runtime.encoderGlitchCount = 0U;
}

/*
 * 读取并变换当前三相测量值。
 * anglePu 在转子未锁定时使用实时角度，锁定后使用固定辨识角度。
 */
static void AutoTune_UpdateMeasurements(fixp30_t anglePu)
{
    FIXP_CosSin_t cosSin;

    Get_RST_Measurements(g_axis.pPWMCHandle,
                         &g_axis.currCtrl.IrstMeas,
                         &g_axis.VotlMeas.VrstMeas);
    Get_Vbus_Measurements(g_axis.pPWMCHandle, &g_axis.busVoltage);

    Clarke_Current(g_axis.currCtrl.IrstMeas, &g_axis.currCtrl.calcIab);
    Clarke_Current(g_axis.VotlMeas.VrstMeas, &g_axis.VotlMeas.calcVab);
    FIXP30_CosSinPU(anglePu, &cosSin);
    Park_Current(g_axis.currCtrl.calcIab, &cosSin, &g_axis.currCtrl.calcIdq);
    Park_Current(g_axis.VotlMeas.calcVab, &cosSin, &g_axis.VotlMeas.calcVdq);
}

/* 将定点 dq 电流恢复为安培，供保护和辨识公式使用。 */
static float AutoTune_GetIdA(void)
{
    return FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE;
}

/* 将定点 q 轴电流恢复为安培。 */
static float AutoTune_GetIqA(void)
{
    return FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE;
}

/* 将定点母线电压恢复为伏特。 */
static float AutoTune_GetVbusV(void)
{
    return FIXP30_toF(g_axis.busVoltage) * VOLTAGE_SCALE;
}

/* Select a lock current that fits the measured resistance and voltage headroom. */
static float AutoTune_SelectIdTargetA(void)
{
    float vbus = AutoTune_GetVbusV();
    float feasibleCurrent;

    if (vbus <= 1.0f || g_rs_ident.Rs <= 0.0f)
    {
        return AUTOTUNE_ID_BIAS_A;
    }

    feasibleCurrent = AUTOTUNE_ALIGN_DUTY_UTILIZATION *
                      0.5f * vbus * AUTOTUNE_ALIGN_MAX_DUTY /
                      g_rs_ident.Rs;
    return AutoTune_Clamp(feasibleCurrent,
                          AUTOTUNE_MIN_ID_BIAS_A,
                          AUTOTUNE_ID_BIAS_A);
}

/* 返回三相电流绝对值中的最大值，用于不依赖 Park 角度的硬限流。 */
static float AutoTune_GetPhaseCurrentPeakA(void)
{
    float ir = fabsf(FIXP30_toF(g_axis.currCtrl.IrstMeas.R) * CURRENT_SCALE);
    float is = fabsf(FIXP30_toF(g_axis.currCtrl.IrstMeas.S) * CURRENT_SCALE);
    float it = fabsf(FIXP30_toF(g_axis.currCtrl.IrstMeas.T) * CURRENT_SCALE);
    float peak = (ir > is) ? ir : is;

    return (it > peak) ? it : peak;
}

/*
 * 将 dq 占空比经过反 Park、反 Clarke 变换后输出到三相 PWM。
 * 这里使用 lockAnglePu，保证辨识期间每次输出都对应同一电角度。
 */
static void AutoTune_WriteDutyDQ(float dutyD, float dutyQ)
{
    Duty_Ddq_t dutyDq;
    Duty_Dab_t dutyAb;
    Duty_Drst_t dutyRst;
    FIXP_CosSin_t cosSin;

    float outputSign = (s_runtime.dqDutySign < 0) ? -1.0f : 1.0f;

    dutyD = AutoTune_Clamp(dutyD, -0.80f, 0.80f) * outputSign;
    dutyQ = AutoTune_Clamp(dutyQ, -0.80f, 0.80f) * outputSign;

    dutyDq.D = FIXP30(dutyD);
    dutyDq.Q = FIXP30(dutyQ);
    FIXP30_CosSinPU(g_rs_ident.lockAnglePu, &cosSin);
    Inv_Park_Duty(dutyDq, &cosSin, &dutyAb);
    Inv_Clarke_Duty(dutyAb, &dutyRst);

    (void)Set_Phase_Duty(g_axis.pPWMCHandle, dutyRst);
}

/*
 * Write the new compare values before enabling the bridge.  TIM1 compare
 * preload means a disabled-to-enabled transition can otherwise expose the
 * previous stage's compare values for one PWM period.
 */
static void AutoTune_ApplyDutyDQ(float dutyD, float dutyQ)
{
    AutoTune_WriteDutyDQ(dutyD, dutyQ);
    SwitchOn_PWM(g_axis.pPWMCHandle);
}

/* 在基础 d 轴偏置上叠加正负高频方波，选择 Ld 或 Lq 注入方向。 */
static void AutoTune_ApplyInjection(bool isDaxis)
{
    float excitation = (float)s_runtime.polarity * s_runtime.hfDuty;

    if (isDaxis)
    {
        AutoTune_ApplyDutyDQ(s_runtime.baseDutyD + excitation, 0.0f);
    }
    else
    {
        AutoTune_ApplyDutyDQ(s_runtime.baseDutyD, excitation);
    }
}

/* Rs 使用两相励磁；方向由正负号选择，占空比由电流调节器缓慢更新。 */
static void AutoTune_ApplyRsDuty(int8_t direction)
{
    uint16_t dutyCount = (uint16_t)(s_runtime.rsDuty *
                                    (float)(PWM_PERIOD_CYCLES >> 1));

    if (direction >= 0)
    {
        SetPWMCompareA(TIM1, dutyCount);
        SetPWMCompareB(TIM1, 0U);
    }
    else
    {
        SetPWMCompareA(TIM1, 0U);
        SetPWMCompareB(TIM1, dutyCount);
    }
    SetPWMCompareC(TIM1, 0U);

    SwitchOn_PWM(g_axis.pPWMCHandle);
    LL_TIM_CC_DisableChannel(TIM1,
                             LL_TIM_CHANNEL_HIGH_C | LL_TIM_CHANNEL_LOW_C);
}

/*
 * 以相电流峰值为反馈调节 Rs 励磁，避免低阻电机被固定占空比直接冲击。
 * 返回 true 表示电流已进入目标窗口，或在最大占空比处达到可用采样电流。
 */
static bool AutoTune_UpdateRsDuty(void)
{
    float phaseCurrent = AutoTune_GetPhaseCurrentPeakA();
    float currentError = AUTOTUNE_RS_TARGET_CURRENT_A - phaseCurrent;
    bool dutySaturated;

    if (g_rs_ident.bAveraging == 0U)
    {
        float dutyStep = AutoTune_Clamp(currentError * AUTOTUNE_RS_DUTY_KI,
                                        -AUTOTUNE_RS_MAX_DUTY_STEP,
                                        AUTOTUNE_RS_MAX_DUTY_STEP);
        s_runtime.rsDuty = AutoTune_Clamp(s_runtime.rsDuty + dutyStep,
                                         0.0f,
                                         AUTOTUNE_RS_MAX_DUTY);
    }

    dutySaturated =
        s_runtime.rsDuty >= (AUTOTUNE_RS_MAX_DUTY - AUTOTUNE_RS_MAX_DUTY_STEP);
    return fabsf(currentError) <= AUTOTUNE_RS_CURRENT_TOL_A ||
           (dutySaturated &&
            phaseCurrent >= AUTOTUNE_RS_MIN_SAMPLE_CURRENT_A);
}

/*
 * 统一故障出口：关闭 PWM、记录故障上下文、设置系统故障状态并打印快照。
 * 该函数不返回，调用者应立即结束当前状态的后续处理。
 */
static void AutoTune_Fail(CurrAutoTuneError_t error)
{
    CurrAutoTuneState_t failedState = g_rs_ident.state;
    float id = AutoTune_GetIdA();
    float iq = AutoTune_GetIqA();
    float ir = FIXP30_toF(g_axis.currCtrl.IrstMeas.R) * CURRENT_SCALE;
    float is = FIXP30_toF(g_axis.currCtrl.IrstMeas.S) * CURRENT_SCALE;
    float it = FIXP30_toF(g_axis.currCtrl.IrstMeas.T) * CURRENT_SCALE;

    SwitchOff_PWM(g_axis.pPWMCHandle);
    g_rs_ident.error = error;
    g_rs_ident.state = CURR_AUTOTUNE_FAULT;
    g_bStartCurrentAutoTune = false;
    g_axis.uFaultOccurred |= MC_SW_ERROR;
    g_axis.state = AXIS_STATE_FAULT_NOW;

    printf("[AUTOTUNE] failed: state=%d, error=%d (%s)\n",
           (int)failedState,
           (int)error,
           AutoTune_ErrorName(error));
    printf("[AUTOTUNE] snapshot: Vbus=%.3f V, Rs=%.6f ohm, "
           "Id=%.3f A, Iq=%.3f A, Ir=%.3f A, Is=%.3f A, It=%.3f A\n",
           AutoTune_GetVbusV(),
           g_rs_ident.Rs,
           id,
           iq,
           ir,
           is,
           it);

    switch (error)
    {
        case CURR_AUTOTUNE_ERROR_RS_INVALID:
            printf("[AUTOTUNE] Rs reject=%d (%s), U+=%.6f V, I+=%.6f A, "
                   "U-=%.6f V, I-=%.6f A, dI=%.6f A, sumI=%.6f A\n",
                   (int)s_rsRejectReason,
                   AutoTune_RsRejectName(s_rsRejectReason),
                   g_rs_ident.U_plus,
                   g_rs_ident.I_plus,
                   g_rs_ident.U_minus,
                   g_rs_ident.I_minus,
                   g_rs_ident.I_plus - g_rs_ident.I_minus,
                   g_rs_ident.I_plus + g_rs_ident.I_minus);
            printf("[AUTOTUNE] Rs state: duty=%.6f, Iphase=%.3f A, "
                   "stable=%lu/%u, averaging=%u, avg=%u/%u\n",
                   s_runtime.rsDuty,
                   AutoTune_GetPhaseCurrentPeakA(),
                   (unsigned long)g_rs_ident.stable_cnt,
                   (unsigned int)AUTOTUNE_RS_STABLE_TICKS,
                   (unsigned int)g_rs_ident.bAveraging,
                   (unsigned int)g_rs_ident.avg_cnt,
                   (unsigned int)AVG_SAMPLE_COUNT);
            break;

        case CURR_AUTOTUNE_ERROR_BIAS_CURRENT:
            printf("[AUTOTUNE] bias: target=%.3f A, measured=%.3f A, "
                   "baseDuty=%.6f, stable=%u/%u, samples=%u, "
                   "ticks=%lu, deadline=%lu, hard=%lu\n",
                   g_rs_ident.idTargetA,
                   g_rs_ident.idBiasA,
                   s_runtime.baseDutyD,
                   (unsigned int)s_runtime.alignStableCount,
                   (unsigned int)AUTOTUNE_ALIGN_STABLE_TICKS,
                   (unsigned int)s_runtime.alignCurrentCount,
                   (unsigned long)s_runtime.stageTick,
                   (unsigned long)AUTOTUNE_ALIGN_MAX_TICKS,
                   (unsigned long)(AUTOTUNE_ALIGN_MAX_TICKS +
                                   AUTOTUNE_ALIGN_STABLE_TICKS));
            printf("[AUTOTUNE] align entry: Iphase=%.3f A, initialized=%u, "
                   "polarityValid=%u, probe=%u/%lu, sign=%d, confirm=%u/%u\n",
                   AutoTune_GetPhaseCurrentPeakA(),
                   (unsigned int)s_runtime.alignInitialized,
                   (unsigned int)s_runtime.alignPolarityValid,
                   (unsigned int)s_runtime.alignProbeTick,
                   (unsigned long)AUTOTUNE_ALIGN_PROBE_MAX_TICKS,
                   (int)s_runtime.dqDutySign,
                   (unsigned int)s_runtime.alignProbeSignCount,
                   (unsigned int)AUTOTUNE_ALIGN_PROBE_SIGN_TICKS);
            break;

        case CURR_AUTOTUNE_ERROR_OVERCURRENT:
            printf("[AUTOTUNE] current limits: phase<=%.3f A, "
                   "|Id|<=%.3f A, |Iq|<=%.3f A\n",
                   AUTOTUNE_MAX_PHASE_CURRENT_A,
                   g_rs_ident.idTargetA + AUTOTUNE_MAX_INJECT_CURRENT_A,
                   AUTOTUNE_MAX_INJECT_CURRENT_A);
            printf("[AUTOTUNE] PWM: CCR_A=%lu, CCR_B=%lu, CCR_C=%lu, "
                   "baseDuty=%.6f, probe=%u/%lu, sign=%d\n",
                   (unsigned long)LL_TIM_OC_GetCompareCH3(TIM1),
                   (unsigned long)LL_TIM_OC_GetCompareCH2(TIM1),
                   (unsigned long)LL_TIM_OC_GetCompareCH1(TIM1),
                   s_runtime.baseDutyD,
                   (unsigned int)s_runtime.alignProbeTick,
                   (unsigned long)AUTOTUNE_ALIGN_PROBE_MAX_TICKS,
                   (int)s_runtime.dqDutySign);
            break;

        case CURR_AUTOTUNE_ERROR_ROTOR_MOVED:
            printf("[AUTOTUNE] rotor: lockRaw=%lu, acceptedRaw=%lu, "
                   "currentRaw=%lu, delta=%lu, total=%lu, sampleStep=%lu\n",
                   (unsigned long)s_runtime.lockRaw,
                   (unsigned long)s_runtime.lastRotorRaw,
                   (unsigned long)g_axis.fbdk.uAngleRaw,
                   (unsigned long)AutoTune_RawAngleDifference(
                       s_runtime.lastRotorRaw, s_runtime.lockRaw),
                   (unsigned long)AutoTune_RawAngleDifference(
                       s_runtime.lastRotorRaw, s_runtime.initialLockRaw),
                   (unsigned long)AutoTune_RawAngleDifference(
                       g_axis.fbdk.uAngleRaw, s_runtime.lastRotorRaw));
            printf("[AUTOTUNE] rotor detector: noise=%lu, trendLimit=%lu, "
                   "moving=%u/%u, stable=%u/%u, rebases=%u, glitches=%u/%u\n",
                   (unsigned long)s_runtime.encoderNoiseRaw,
                   (unsigned long)AutoTune_MotionTrendLimitRaw(),
                   (unsigned int)s_runtime.movingWindowCount,
                   (unsigned int)AUTOTUNE_MOTION_CONFIRM_WINDOWS,
                   (unsigned int)s_runtime.stationaryWindowCount,
                   (unsigned int)AUTOTUNE_REBASE_STABLE_WINDOWS,
                   (unsigned int)s_runtime.rotorRebaseCount,
                   (unsigned int)s_runtime.encoderGlitchCount,
                   (unsigned int)AUTOTUNE_ENCODER_GLITCH_TICKS);
            break;

        case CURR_AUTOTUNE_ERROR_LD_INVALID:
        case CURR_AUTOTUNE_ERROR_LQ_INVALID:
            printf("[AUTOTUNE] %s reject=%d (%s), cycles=%u, samples=%u, "
                   "retained=%u, motionReject=%u, duty=%.6f, Ipp=%.4f A\n",
                   (error == CURR_AUTOTUNE_ERROR_LD_INVALID) ? "Ld" : "Lq",
                   (int)s_lRejectReason,
                   AutoTune_LRejectName(s_lRejectReason),
                   (unsigned int)s_runtime.completedCycles,
                   (unsigned int)s_runtime.estimateCount,
                   (unsigned int)s_runtime.lastRetainedCount,
                   (unsigned int)s_runtime.motionRejectedCount,
                   s_runtime.hfDuty,
                   s_runtime.lastCurrentPeakToPeak);
            printf("[AUTOTUNE] L stats: median=%.3f uH, CV=%.3f, "
                   "pos=%u/%.3f uH, neg=%u/%.3f uH\n",
                   s_runtime.lastMedian * 1000000.0f,
                   s_runtime.lastCoefficientOfVariation,
                   (unsigned int)s_runtime.positiveEstimateCount,
                   AutoTune_SafeMean(s_runtime.positiveEstimateSum,
                                     s_runtime.positiveEstimateCount) * 1000000.0f,
                   (unsigned int)s_runtime.negativeEstimateCount,
                   AutoTune_SafeMean(s_runtime.negativeEstimateSum,
                                     s_runtime.negativeEstimateCount) * 1000000.0f);
            printf("[AUTOTUNE] L motion: maxSegmentDrift=%lu raw, "
                   "limit=%lu raw\n",
                   (unsigned long)s_runtime.maxObservedSegmentDriftRaw,
                   (unsigned long)AutoTune_SegmentDriftRaw(
                       (error == CURR_AUTOTUNE_ERROR_LD_INVALID) ?
                           AUTOTUNE_LD_SEGMENT_DRIFT_DEG :
                           AUTOTUNE_LQ_SEGMENT_DRIFT_DEG));
            break;

        default:
            break;
    }
}

/* 所有通电辨识阶段都执行，与 dq 角度无关，作为最外层电流保护。 */
static bool AutoTune_CheckPhaseCurrent(void)
{
    if (AutoTune_GetPhaseCurrentPeakA() > AUTOTUNE_MAX_PHASE_CURRENT_A)
    {
        AutoTune_Fail(CURR_AUTOTUNE_ERROR_OVERCURRENT);
        return false;
    }
    return true;
}

/*
 * 检查电流和转子角度保护。
 * 返回 true 表示可以继续当前注入；返回 false 表示已进入故障状态。
 * 单点跳变按编码器异常处理；连续同向运动才故障，一次性回弹稳定后重定基准。
 */
static bool AutoTune_CheckProtection(void)
{
    float id = AutoTune_GetIdA();
    float iq = AutoTune_GetIqA();
    uint32_t currentRaw;
    uint32_t sampleDelta;
    uint32_t rotorDelta;
    uint32_t sampleStepLimit;
    uint32_t trendLimit;
    uint32_t totalRebaseLimit;
    int32_t signedWindowDelta;
    int32_t signedFromInitial;
    int8_t windowDirection;
    bool movingAway;

    if ((fabsf(id) > (g_rs_ident.idTargetA + AUTOTUNE_MAX_INJECT_CURRENT_A)) ||
        (fabsf(iq) > AUTOTUNE_MAX_INJECT_CURRENT_A))
    {
        AutoTune_Fail(CURR_AUTOTUNE_ERROR_OVERCURRENT);
        return false;
    }

    if (!s_runtime.lockAngleValid)
    {
        return true;
    }

    currentRaw = g_axis.fbdk.uAngleRaw;
    sampleDelta = AutoTune_RawAngleDifference(currentRaw,
                                               s_runtime.lastRotorRaw);
    sampleStepLimit = AutoTune_MaxU32(
        AutoTune_RotorDriftRaw(AUTOTUNE_ENCODER_MAX_STEP_DEG),
        AutoTune_EncoderDifferenceNoiseRaw());
    if (sampleDelta > sampleStepLimit)
    {
        if (s_runtime.encoderGlitchCount < UINT16_MAX)
        {
            s_runtime.encoderGlitchCount++;
        }
        if (s_runtime.encoderGlitchCount >= AUTOTUNE_ENCODER_GLITCH_TICKS)
        {
            AutoTune_Fail(CURR_AUTOTUNE_ERROR_ROTOR_MOVED);
            return false;
        }
        return true;
    }

    s_runtime.lastRotorRaw = currentRaw;
    s_runtime.encoderGlitchCount = 0U;
    s_runtime.motionWindowTick++;
    if (s_runtime.motionWindowTick < AUTOTUNE_MOTION_WINDOW_TICKS)
    {
        return true;
    }

    signedWindowDelta = AutoTune_RawAngleSignedDifference(
        s_runtime.lastRotorRaw, s_runtime.motionWindowStartRaw);
    trendLimit = AutoTune_MotionTrendLimitRaw();
    windowDirection = (signedWindowDelta > (int32_t)trendLimit) ? 1 :
                      (signedWindowDelta < -(int32_t)trendLimit) ? -1 : 0;
    signedFromInitial = AutoTune_RawAngleSignedDifference(
        s_runtime.lastRotorRaw, s_runtime.initialLockRaw);
    movingAway =
        ((windowDirection > 0) && (signedFromInitial > (int32_t)trendLimit)) ||
        ((windowDirection < 0) && (signedFromInitial < -(int32_t)trendLimit));
    s_runtime.motionWindowStartRaw = s_runtime.lastRotorRaw;
    s_runtime.motionWindowTick = 0U;

    if (windowDirection != 0)
    {
        if (movingAway && windowDirection == s_runtime.motionDirection)
        {
            if (s_runtime.movingWindowCount < UINT8_MAX)
            {
                s_runtime.movingWindowCount++;
            }
        }
        else if (movingAway)
        {
            s_runtime.motionDirection = windowDirection;
            s_runtime.movingWindowCount = 1U;
        }
        else
        {
            s_runtime.motionDirection = windowDirection;
            s_runtime.movingWindowCount = 0U;
        }
        s_runtime.stationaryWindowCount = 0U;
        if (s_runtime.movingWindowCount >= AUTOTUNE_MOTION_CONFIRM_WINDOWS)
        {
            AutoTune_Fail(CURR_AUTOTUNE_ERROR_ROTOR_MOVED);
            return false;
        }
    }
    else
    {
        s_runtime.motionDirection = 0;
        s_runtime.movingWindowCount = 0U;
        if (s_runtime.stationaryWindowCount < UINT8_MAX)
        {
            s_runtime.stationaryWindowCount++;
        }

        rotorDelta = AutoTune_RawAngleDifference(s_runtime.lastRotorRaw,
                                                  s_runtime.lockRaw);
        if (s_runtime.stationaryWindowCount >= AUTOTUNE_REBASE_STABLE_WINDOWS &&
            rotorDelta > AutoTune_EncoderDifferenceNoiseRaw())
        {
            totalRebaseLimit = AutoTune_RotorDriftRaw(AUTOTUNE_MAX_REBASE_ELEC_DEG);
            if (AutoTune_RawAngleDifference(s_runtime.lastRotorRaw,
                                             s_runtime.initialLockRaw) > totalRebaseLimit)
            {
                AutoTune_Fail(CURR_AUTOTUNE_ERROR_ROTOR_MOVED);
                return false;
            }
            s_runtime.lockRaw = s_runtime.lastRotorRaw;
            s_runtime.stationaryWindowCount = 0U;
            if (s_runtime.rotorRebaseCount < UINT16_MAX)
            {
                s_runtime.rotorRebaseCount++;
            }
            printf("[AUTOTUNE] rotor rebase: raw=%lu, shift=%lu, total=%lu, "
                   "noise=%lu\n",
                   (unsigned long)s_runtime.lockRaw,
                   (unsigned long)rotorDelta,
                   (unsigned long)AutoTune_RawAngleDifference(
                       s_runtime.lockRaw, s_runtime.initialLockRaw),
                   (unsigned long)s_runtime.encoderNoiseRaw);
        }
    }

    return true;
}

/*
 * Rs 单方向采样器：调用方保证相电流处于目标窗口，本函数按连续保持时间
 * 进入平均。开始平均后占空比保持不变，避免 PWM 纹波反复清空采样窗口。
 * 返回 1 表示本方向的平均值已经写入 currentOut/voltageOut。
 */
static uint8_t AutoTune_SampleRs(float currentMeas,
                                 float voltageMeas,
                                 float *currentOut,
                                 float *voltageOut)
{
    if (g_rs_ident.bAveraging == 0U)
    {
        if (g_rs_ident.stable_cnt < UINT32_MAX)
        {
            g_rs_ident.stable_cnt++;
        }
        if (g_rs_ident.stable_cnt >= AUTOTUNE_RS_STABLE_TICKS)
        {
            g_rs_ident.bAveraging = 1U;
            g_rs_ident.avg_cnt = 0U;
            g_rs_ident.sum_U = 0.0f;
            g_rs_ident.sum_I = 0.0f;
            g_rs_ident.stable_cnt = 0U;
        }
        *currentOut = currentMeas;
        return 0U;
    }

    g_rs_ident.sum_U += voltageMeas;
    g_rs_ident.sum_I += currentMeas;
    g_rs_ident.avg_cnt++;
    if (g_rs_ident.avg_cnt >= AVG_SAMPLE_COUNT)
    {
        *voltageOut = g_rs_ident.sum_U / (float)AVG_SAMPLE_COUNT;
        *currentOut = g_rs_ident.sum_I / (float)AVG_SAMPLE_COUNT;
        g_rs_ident.bAveraging = 0U;
        return 1U;
    }
    return 0U;
}

/* 清空 Rs 稳态检测、平均计数和累加器，准备新的正向或反向测试。 */
static void AutoTune_ResetRsSampler(void)
{
    g_rs_ident.stable_cnt = 0U;
    g_rs_ident.bAveraging = 0U;
    g_rs_ident.avg_cnt = 0U;
    g_rs_ident.sum_U = 0.0f;
    g_rs_ident.sum_I = 0.0f;
}

/*
 * 根据正、负直流测量值计算 Rs。
 * 优先使用差分公式消除偏置；电流差过小时改用求和公式，并检查结果范围。
 */
static bool AutoTune_CalculateRs(void)
{
    float denominator = g_rs_ident.I_plus - g_rs_ident.I_minus;
    float rs;

    s_rsRejectReason = AUTOTUNE_RS_REJECT_NONE;

    if (fabsf(denominator) > 0.10f)
    {
        rs = (g_rs_ident.U_plus - g_rs_ident.U_minus) / denominator;
    }
    else
    {
        denominator = g_rs_ident.I_plus + g_rs_ident.I_minus;
        if (fabsf(denominator) <= 0.10f)
        {
            s_rsRejectReason = AUTOTUNE_RS_REJECT_CURRENT_DELTA;
            return false;
        }
        rs = (g_rs_ident.U_plus + g_rs_ident.U_minus) / denominator;
    }

    rs = fabsf(rs);
    if (!isfinite(rs) || rs < AUTOTUNE_MIN_RS_OHM || rs > AUTOTUNE_MAX_RS_OHM)
    {
        s_rsRejectReason = AUTOTUNE_RS_REJECT_RESULT_RANGE;
        return false;
    }

    g_rs_ident.Rs = rs;
    g_axis.fRs = rs;
    return true;
}

/*
 * 初始化一个新的 Ld/Lq 注入测量窗口。
 * 同时清空半周期积分、正负样本统计和异常样本计数。
 */
static void AutoTune_BeginInjection(float dutyAmplitude)
{
    s_runtime.polarity = 1;
    s_runtime.halfTick = 0U;
    s_runtime.completedCycles = 0U;
    s_runtime.segmentActive = false;
    s_runtime.segmentFlux = 0.0f;
    s_runtime.segmentStartRaw = 0U;
    s_runtime.segmentMaxDriftRaw = 0U;
    s_runtime.maxObservedSegmentDriftRaw = 0U;
    s_runtime.motionRejectedCount = 0U;
    s_runtime.cycleMinCurrent = 1.0e30f;
    s_runtime.cycleMaxCurrent = -1.0e30f;
    s_runtime.lastCurrentPeakToPeak = 0.0f;
    s_runtime.hfDuty = AutoTune_Clamp(dutyAmplitude,
                                      AUTOTUNE_MIN_HF_DUTY,
                                      AUTOTUNE_MAX_HF_DUTY);
    s_runtime.adaptStableCount = 0U;
    s_runtime.estimateCount = 0U;
    s_runtime.positiveEstimateSum = 0.0f;
    s_runtime.negativeEstimateSum = 0.0f;
    s_runtime.positiveEstimateCount = 0U;
    s_runtime.negativeEstimateCount = 0U;
    s_runtime.lastMedian = 0.0f;
    s_runtime.lastCoefficientOfVariation = 0.0f;
    s_runtime.lastRetainedCount = 0U;
    s_lRejectReason = AUTOTUNE_L_REJECT_NONE;
    memset(s_runtime.estimates, 0, sizeof(s_runtime.estimates));
}

/*
 * 对一个注入轴的全部半周期样本做最终统计：
 * 正负极性检查 -> 排序取中值 -> 中值窗口剔除 -> 均值和 CV 校验。
 */
static bool AutoTune_CalculateInductance(float *result)
{
    uint16_t count = s_runtime.estimateCount;
    uint16_t validCount = 0U;
    float median;
    float sum = 0.0f;
    float sumSquared = 0.0f;

    s_lRejectReason = AUTOTUNE_L_REJECT_NONE;

    /* 步骤 1：保证总样本数以及正、负半周期样本数足够。 */
    if (result == NULL || count < 8U)
    /* 步骤 2：比较正、负半周期均值，排除明显的极性不对称。 */
    {
        s_lRejectReason = AUTOTUNE_L_REJECT_SAMPLE_COUNT;
        return false;
    }

    if (s_runtime.positiveEstimateCount < 4U ||
        s_runtime.negativeEstimateCount < 4U)
    {
        s_lRejectReason = AUTOTUNE_L_REJECT_POLARITY_COUNT;
        return false;
    }

    {
        float positiveMean = s_runtime.positiveEstimateSum /
                             (float)s_runtime.positiveEstimateCount;
        float negativeMean = s_runtime.negativeEstimateSum /
                             (float)s_runtime.negativeEstimateCount;
        float largerMean = (positiveMean > negativeMean) ? positiveMean : negativeMean;

        if (largerMean <= 0.0f ||
            fabsf(positiveMean - negativeMean) > (0.50f * largerMean))
        {
            s_lRejectReason = AUTOTUNE_L_REJECT_POLARITY_MISMATCH;
            return false;
        }
    }

    /* 步骤 3：原地插入排序，为中值筛选准备有序样本。 */
    for (uint16_t i = 1U; i < count; i++)
    {
        float value = s_runtime.estimates[i];
        uint16_t j = i;
        while (j > 0U && s_runtime.estimates[j - 1U] > value)
        {
            s_runtime.estimates[j] = s_runtime.estimates[j - 1U];
            j--;
        }
        s_runtime.estimates[j] = value;
    }

    /* 偶数样本时当前实现取上中位位置，作为异常值筛选中心。 */
    median = s_runtime.estimates[count / 2U];
    s_runtime.lastMedian = median;
    if (!isfinite(median) || median < AUTOTUNE_MIN_L_H || median > AUTOTUNE_MAX_L_H)
    {
        s_lRejectReason = AUTOTUNE_L_REJECT_MEDIAN_RANGE;
        return false;
    }

    /* 步骤 4：仅保留中值 ±35% 范围内的样本。 */
    for (uint16_t i = 0U; i < count; i++)
    {
        float value = s_runtime.estimates[i];
        if (fabsf(value - median) <= (0.35f * median))
        {
            sum += value;
            sumSquared += value * value;
            validCount++;
        }
    }

    if (validCount < 8U)
    {
        s_runtime.lastRetainedCount = validCount;
        s_lRejectReason = AUTOTUNE_L_REJECT_RETAINED_COUNT;
        return false;
    }

    /* 步骤 5：使用保留样本的均值作为最终电感。 */
    s_runtime.lastRetainedCount = validCount;
    *result = sum / (float)validCount;
    if (*result < AUTOTUNE_MIN_L_H || *result > AUTOTUNE_MAX_L_H)
    {
        s_lRejectReason = AUTOTUNE_L_REJECT_RESULT_RANGE;
        return false;
    }

    /* 步骤 6：计算变异系数，离散度过大时拒绝本次结果。 */
    {
        float variance = sumSquared / (float)validCount - (*result * *result);
        float coefficientOfVariation;
        if (variance < 0.0f)
        {
            variance = 0.0f;
        }
        coefficientOfVariation = sqrtf(variance) / *result;
        s_runtime.lastCoefficientOfVariation = coefficientOfVariation;
        if (coefficientOfVariation > 0.40f)
        {
            s_lRejectReason = AUTOTUNE_L_REJECT_SCATTER;
            return false;
        }
    }

    return true;
}

static bool AutoTune_HasMinimumInductanceSamples(void)
{
    return s_runtime.estimateCount >= 8U &&
           s_runtime.positiveEstimateCount >= 4U &&
           s_runtime.negativeEstimateCount >= 4U;
}

/*
 * 根据实测 Ipp 调整 d/q 轴注入幅值。返回 true 表示本周期已落入目标窗口。
 * 比例修正限制在 0.5 到 2 倍，避免电感跨度很大时占空比跳变过猛。
 */
static bool AutoTune_AdjustInjectionAmplitude(float currentPeakToPeak)
{
    float scale;

    if (currentPeakToPeak >= AUTOTUNE_MIN_CURRENT_PP_A &&
        currentPeakToPeak <= AUTOTUNE_MAX_CURRENT_PP_A)
    {
        return true;
    }

    if (currentPeakToPeak > 0.02f)
    {
        scale = AUTOTUNE_TARGET_CURRENT_PP_A / currentPeakToPeak;
        scale = AutoTune_Clamp(scale, 0.5f, 2.0f);
    }
    else
    {
        scale = 2.0f;
    }

    s_runtime.hfDuty = AutoTune_Clamp(s_runtime.hfDuty * scale,
                                      AUTOTUNE_MIN_HF_DUTY,
                                      AUTOTUNE_MAX_HF_DUTY);
    return false;
}

/*
 * 执行一个 PWM 周期的方波注入和伏秒积分。
 * 一个半周期结束时计算一个 Ld/Lq 样本，并在负半周期结束时完成一个完整周期。
 */
static bool AutoTune_RunInjection(bool isDaxis, bool collectEstimates)
{
    const float samplePeriod = 1.0f / (float)PWM_FREQUENCY;
    const uint16_t halfTicks = isDaxis ?
        AUTOTUNE_LD_INJECT_HALF_TICKS : AUTOTUNE_LQ_INJECT_HALF_TICKS;
    const uint32_t segmentDriftLimit = AutoTune_SegmentDriftRaw(isDaxis ?
        AUTOTUNE_LD_SEGMENT_DRIFT_DEG : AUTOTUNE_LQ_SEGMENT_DRIFT_DEG);
    /* Ld 使用去除直流偏置后的 Id；Lq 直接使用 Iq。 */
    float axisCurrent = isDaxis ?
        (AutoTune_GetIdA() - g_rs_ident.idBiasA) : AutoTune_GetIqA();
    bool cycleCompleted;

    if (!AutoTune_CheckProtection())
    {
        return false;
    }

    /* 记录一个完整正负周期内的最大、最小电流，用于 Ipp 幅值自适应。 */
    if (axisCurrent < s_runtime.cycleMinCurrent)
    {
        s_runtime.cycleMinCurrent = axisCurrent;
    }
    if (axisCurrent > s_runtime.cycleMaxCurrent)
    {
        s_runtime.cycleMaxCurrent = axisCurrent;
    }

    /*
     * 极性切换后的前几个 PWM 周期作为空白区。
     * 空白区结束时记录电流和编码器起点，之后才开始伏秒积分。
     */
    if (s_runtime.halfTick == AUTOTUNE_INJECT_BLANK_TICKS)
    {
        s_runtime.segmentStartCurrent = axisCurrent;
        s_runtime.segmentPrevCurrent = axisCurrent;
        s_runtime.segmentFlux = 0.0f;
        s_runtime.segmentStartRaw = s_runtime.lastRotorRaw;
        s_runtime.segmentMaxDriftRaw = 0U;
        s_runtime.segmentActive = true;
    }
    else if ((s_runtime.halfTick > AUTOTUNE_INJECT_BLANK_TICKS) &&
             s_runtime.segmentActive)
    {
        /*
         * 梯形近似本采样区间的电流，并累加：
         * segmentFlux = integral(u_inj - Rs*i)dt。
         */
        float currentAverage = 0.5f * (s_runtime.segmentPrevCurrent + axisCurrent);
        float excitationVoltage = (float)s_runtime.polarity * s_runtime.hfDuty *
                                  0.5f * AutoTune_GetVbusV();
        s_runtime.segmentFlux +=
            (excitationVoltage - g_rs_ident.Rs * currentAverage) * samplePeriod;
        s_runtime.segmentPrevCurrent = axisCurrent;

        /* 同时记录本半周期的最大转子漂移，供样本级运动剔除使用。 */
        {
            uint32_t segmentDrift = AutoTune_RawAngleDifference(
                s_runtime.lastRotorRaw, s_runtime.segmentStartRaw);
            if (segmentDrift > s_runtime.segmentMaxDriftRaw)
            {
                s_runtime.segmentMaxDriftRaw = segmentDrift;
            }
        }
    }

    s_runtime.halfTick++;
    if (s_runtime.halfTick < halfTicks)
    {
        AutoTune_ApplyInjection(isDaxis);
        return false;
    }

    /*
     * 半周期结束：只有转子漂移合格且 |DeltaI| 足够大时才计算样本。
     * L = segmentFlux / (i_end - i_start)。
     */
    if (collectEstimates && s_runtime.segmentActive &&
        s_runtime.segmentMaxDriftRaw <= segmentDriftLimit)
    {
        float deltaCurrent = axisCurrent - s_runtime.segmentStartCurrent;
        if (fabsf(deltaCurrent) > 0.05f)
        {
            float inductance = s_runtime.segmentFlux / deltaCurrent;
            if (isfinite(inductance) &&
                inductance >= AUTOTUNE_MIN_L_H &&
                inductance <= AUTOTUNE_MAX_L_H &&
                s_runtime.estimateCount < AUTOTUNE_MAX_ESTIMATES)
            {
                s_runtime.estimates[s_runtime.estimateCount++] = inductance;
                /* 正、负半周期分别统计，用于最终极性一致性检查。 */
                if (s_runtime.polarity > 0)
                {
                    s_runtime.positiveEstimateSum += inductance;
                    s_runtime.positiveEstimateCount++;
                }
                else
                {
                    s_runtime.negativeEstimateSum += inductance;
                    s_runtime.negativeEstimateCount++;
                }
            }
        }
    }
    else if (collectEstimates && s_runtime.segmentActive &&
             s_runtime.segmentMaxDriftRaw > segmentDriftLimit)
    {
        s_runtime.motionRejectedCount++;
    }

    if (s_runtime.segmentMaxDriftRaw >
        s_runtime.maxObservedSegmentDriftRaw)
    {
        s_runtime.maxObservedSegmentDriftRaw =
            s_runtime.segmentMaxDriftRaw;
    }

    /* 负半周期结束表示一个完整注入周期完成，然后翻转到下一极性。 */
    cycleCompleted = (s_runtime.polarity < 0);
    s_runtime.polarity = (int8_t)-s_runtime.polarity;
    s_runtime.halfTick = 0U;
    s_runtime.segmentActive = false;

    /* 完整周期结束后更新 Ipp；预热阶段可据此调整 Ld 注入幅值。 */
    if (cycleCompleted)
    {
        float currentPeakToPeak = s_runtime.cycleMaxCurrent - s_runtime.cycleMinCurrent;
        s_runtime.lastCurrentPeakToPeak = currentPeakToPeak;
        s_runtime.completedCycles++;
        if (!collectEstimates)
        {
            if (AutoTune_AdjustInjectionAmplitude(currentPeakToPeak))
            {
                if (s_runtime.adaptStableCount < UINT16_MAX)
                {
                    s_runtime.adaptStableCount++;
                }
            }
            else
            {
                s_runtime.adaptStableCount = 0U;
            }
        }
        s_runtime.cycleMinCurrent = 1.0e30f;
        s_runtime.cycleMaxCurrent = -1.0e30f;
    }

    AutoTune_ApplyInjection(isDaxis);
    return cycleCompleted;
}

void CurrAutoTune_Start(void)
{
    /* 重新开始时清空上一次的结果、状态机计数和半周期缓存。 */
    memset(&g_rs_ident, 0, sizeof(g_rs_ident));
    memset(&s_runtime, 0, sizeof(s_runtime));
    s_runtime.dqDutySign = 1;
    g_rs_ident.state = CURR_AUTOTUNE_IDLE;
    g_rs_ident.error = CURR_AUTOTUNE_ERROR_NONE;
    g_rs_ident.idTargetA = AUTOTUNE_ID_BIAS_A;
    g_axis.posCtrl.bCalibFlag = false;
    g_axis.posCtrl.uCalibCount = 0U;

    printf("[AUTOTUNE] start: PWM=%lu Hz, IdBias=%.3f A, "
           "RsTarget=%.3f A, LdInject=%lu Hz, LqInject=%lu Hz, "
           "IppTarget=%.3f A\n",
           (unsigned long)PWM_FREQUENCY,
           g_rs_ident.idTargetA,
           AUTOTUNE_RS_TARGET_CURRENT_A,
           (unsigned long)(PWM_FREQUENCY /
               (2U * AUTOTUNE_LD_INJECT_HALF_TICKS)),
            (unsigned long)(PWM_FREQUENCY /
                (2U * AUTOTUNE_LQ_INJECT_HALF_TICKS)),
            AUTOTUNE_TARGET_CURRENT_PP_A);
}

void CurrAutoTune_Abort(void)
{
    /* 用户主动取消时必须先撤销 PWM，再清空运行时状态。 */
    SwitchOff_PWM(g_axis.pPWMCHandle);
    memset(&s_runtime, 0, sizeof(s_runtime));
    g_rs_ident.state = CURR_AUTOTUNE_IDLE;
    g_rs_ident.error = CURR_AUTOTUNE_ERROR_NONE;
    g_bStartCurrentAutoTune = false;

    printf("[AUTOTUNE] aborted\n");
}

void CurrAutoTune_Handle(void)
{
    fixp30_t measurementAngle;
    bool rsRateTick = false;

    /* 第一步：按当前阶段选择实时角度或固定锁定角度并更新测量值。 */
    if (s_runtime.lockAngleValid)
    {
        measurementAngle = g_rs_ident.lockAnglePu;
    }
    else
    {
        Get_Angle(&measurementAngle);
    }
    AutoTune_UpdateMeasurements(measurementAngle);

    if (g_rs_ident.state >= CURR_AUTOTUNE_RS_ROTOR_LOCK &&
        g_rs_ident.state <= CURR_AUTOTUNE_LQ_MEASURE &&
        !(g_rs_ident.state == CURR_AUTOTUNE_L_ALIGN &&
          !s_runtime.alignInitialized) &&
        !AutoTune_CheckPhaseCurrent())
    {
        return;
    }

    /* IDLE 只负责把状态机推进到首次 RS_LOCK，不在此处执行实际通电。 */
    if (g_rs_ident.state == CURR_AUTOTUNE_IDLE)
    {
        /*
         * Prime known-safe raw compares before the first two-phase pulse.
         * RS_LOCK is rate divided, so these preloads latch while PWM remains
         * disabled and cannot expose compare values from the previous mode.
         */
        SwitchOff_PWM(g_axis.pPWMCHandle);
        SetPWMCompareA(TIM1, 0U);
        SetPWMCompareB(TIM1, 0U);
        SetPWMCompareC(TIM1, 0U);
        AutoTune_ResetRsSampler();
        s_runtime.stageTick = 0U;
        s_runtime.rsRateDivider = 0U;
        s_runtime.rsDuty = 0.0f;
        g_rs_ident.state = CURR_AUTOTUNE_RS_ROTOR_LOCK;
        printf("[AUTOTUNE] stage=RS_LOCK, target=%.3f A, maxDuty=%.4f\n",
               AUTOTUNE_RS_TARGET_CURRENT_A,
               AUTOTUNE_RS_MAX_DUTY);
        return;
    }

    /* Rs 阶段降频执行，避免稳态采样和日志状态被 PWM 频率过度刷新。 */
    if (g_rs_ident.state <= CURR_AUTOTUNE_RS_NEG)
    {
        s_runtime.rsRateDivider++;
        if (s_runtime.rsRateDivider >= AUTOTUNE_RS_RATE_DIVIDER)
        {
            s_runtime.rsRateDivider = 0U;
            rsRateTick = true;
        }
        if (!rsRateTick)
        {
            return;
        }
    }

    switch (g_rs_ident.state)
    {
        /* RS_LOCK：缓慢建立目标两相直流电流，让转子进入可重复状态。 */
        case CURR_AUTOTUNE_RS_ROTOR_LOCK:
            (void)AutoTune_UpdateRsDuty();
            AutoTune_ApplyRsDuty(1);
            s_runtime.stageTick++;
            if (s_runtime.stageTick >= AUTOTUNE_RS_LOCK_TICKS)
            {
                s_runtime.stageTick = 0U;
                AutoTune_ResetRsSampler();
                g_rs_ident.state = CURR_AUTOTUNE_RS_POS;
                printf("[AUTOTUNE] stage=RS_POS, duty=%.6f, Iphase=%.3f A\n",
                       s_runtime.rsDuty,
                       AutoTune_GetPhaseCurrentPeakA());
            }
            break;

        /* RS_POS：等待正向电流稳定，并对 Vd/Id 做稳态平均。 */
        case CURR_AUTOTUNE_RS_POS:
        {
            float current = AutoTune_GetIdA();
            float voltage = FIXP30_toF(g_axis.VotlMeas.calcVdq.D) * VOLTAGE_SCALE;
            bool currentReady = AutoTune_UpdateRsDuty();

            AutoTune_ApplyRsDuty(1);
            s_runtime.stageTick++;

            if (!currentReady && g_rs_ident.bAveraging == 0U)
            {
                AutoTune_ResetRsSampler();
                g_rs_ident.I_plus = current;
            }
            else if (AutoTune_SampleRs(current,
                                       voltage,
                                       &g_rs_ident.I_plus,
                                       &g_rs_ident.U_plus) != 0U)
            {
                s_runtime.stageTick = 0U;
                s_runtime.rsDuty = 0.0f;
                AutoTune_ApplyRsDuty(-1);
                AutoTune_ResetRsSampler();
                g_rs_ident.state = CURR_AUTOTUNE_RS_NEG;
                printf("[AUTOTUNE] RS_POS done: U=%.5f V, I=%.5f A\n",
                       g_rs_ident.U_plus,
                       g_rs_ident.I_plus);
                printf("[AUTOTUNE] stage=RS_NEG\n");
            }

            if (s_runtime.stageTick >= AUTOTUNE_RS_STAGE_MAX_TICKS)
            {
                s_rsRejectReason = AUTOTUNE_RS_REJECT_CURRENT_DELTA;
                AutoTune_Fail(CURR_AUTOTUNE_ERROR_RS_INVALID);
            }
            break;
        }

        /* RS_NEG：反向通电并采集第二组稳态数据，然后计算 Rs。 */
        case CURR_AUTOTUNE_RS_NEG:
        {
            float current = AutoTune_GetIdA();
            float voltage = FIXP30_toF(g_axis.VotlMeas.calcVdq.D) * VOLTAGE_SCALE;
            bool currentReady = AutoTune_UpdateRsDuty();

            AutoTune_ApplyRsDuty(-1);
            s_runtime.stageTick++;

            if (!currentReady && g_rs_ident.bAveraging == 0U)
            {
                AutoTune_ResetRsSampler();
                g_rs_ident.I_minus = current;
            }
            else if (AutoTune_SampleRs(current,
                                       voltage,
                                       &g_rs_ident.I_minus,
                                       &g_rs_ident.U_minus) != 0U)
            {
                if (!AutoTune_CalculateRs())
                {
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_RS_INVALID);
                    break;
                }

                g_rs_ident.idTargetA = AutoTune_SelectIdTargetA();

                SwitchOff_PWM(g_axis.pPWMCHandle);
                /*
                 * RS uses raw A/B compares and leaves C disabled.  Prime a
                 * balanced zero vector while the bridge remains off; the
                 * following decay window gives the preload time to latch
                 * before L_ALIGN enables all three phases.
                 */
                AutoTune_WriteDutyDQ(0.0f, 0.0f);
                s_runtime.stageTick = 0U;
                s_runtime.lockAngleValid = false;
                s_runtime.alignInitialized = false;
                s_runtime.alignPolarityValid = false;
                s_runtime.alignProbeDecay = false;
                s_runtime.alignProbeTick = 0U;
                s_runtime.alignProbeSignCount = 0U;
                s_runtime.alignProbeSignCandidate = 0;
                s_runtime.dqDutySign = 1;
                s_runtime.alignEntryStableCount = 0U;
                g_rs_ident.state = CURR_AUTOTUNE_L_ALIGN;
                printf("[AUTOTUNE] RS_NEG done: U=%.5f V, I=%.5f A\n",
                       g_rs_ident.U_minus,
                       g_rs_ident.I_minus);
                printf("[AUTOTUNE] Rs done: %.6f ohm\n", g_rs_ident.Rs);
                printf("[AUTOTUNE] L_ALIGN target: Id=%.3f A\n",
                       g_rs_ident.idTargetA);
                printf("[AUTOTUNE] stage=L_ALIGN, waiting for current decay\n");
            }

            if (s_runtime.stageTick >= AUTOTUNE_RS_STAGE_MAX_TICKS)
            {
                s_rsRejectReason = AUTOTUNE_RS_REJECT_CURRENT_DELTA;
                AutoTune_Fail(CURR_AUTOTUNE_ERROR_RS_INVALID);
            }
            break;
        }

        /*
         * L_ALIGN：回到固定电角度零点，建立 Id 偏置。
         * 只有 Id、Iq 同时稳定后才开始 Ld 高频注入。
         */
        case CURR_AUTOTUNE_L_ALIGN:
        {
            float id = AutoTune_GetIdA();
            float iq = AutoTune_GetIqA();
            float vbus = AutoTune_GetVbusV();
            float currentError;
            bool alignmentReady = false;

            /*
             * Rs 负向采样结束后先保持 PWM 关闭，等待续流和采样瞬态消失。
             * 只有三相电流连续低于入口阈值后，才允许重新施加 d 轴电压。
             */
            if (!s_runtime.alignInitialized)
            {
                SwitchOff_PWM(g_axis.pPWMCHandle);
                s_runtime.stageTick++;
                if (AutoTune_GetPhaseCurrentPeakA() <=
                    AUTOTUNE_ALIGN_ENTRY_CURRENT_A)
                {
                    if (s_runtime.alignEntryStableCount < UINT16_MAX)
                    {
                        s_runtime.alignEntryStableCount++;
                    }
                }
                else
                {
                    s_runtime.alignEntryStableCount = 0U;
                }

                if (s_runtime.alignEntryStableCount <
                    AUTOTUNE_ALIGN_ENTRY_STABLE_TICKS)
                {
                    if (s_runtime.stageTick >= AUTOTUNE_ALIGN_ENTRY_MAX_TICKS)
                    {
                        g_rs_ident.idBiasA = id;
                        AutoTune_Fail(CURR_AUTOTUNE_ERROR_BIAS_CURRENT);
                    }
                    break;
                }

                /*
                 * 编码器校准将电角度零点定义为固定的定子 d 轴矢量。
                 * 两相 Rs 测试完成后回到同一矢量，确保每次 Ld/Lq 辨识
                 * 使用相同的 PWM 扇区和逆变器死区模式。
                 */
                g_rs_ident.lockAnglePu = FIXP30(0.0f);
                s_runtime.lockAngleValid = true;
                s_runtime.lockRaw = g_axis.fbdk.uAngleRaw;
                s_runtime.baseDutyD = AUTOTUNE_ALIGN_PROBE_DUTY;
                s_runtime.alignCurrentSum = 0.0f;
                s_runtime.alignCurrentCount = 0U;
                s_runtime.alignStableCount = 0U;
                s_runtime.stageTick = 0U;
                s_runtime.alignProbeTick = 0U;
                s_runtime.alignProbeSignCount = 0U;
                s_runtime.alignProbeSignCandidate = 0;
                s_runtime.alignPolarityValid = false;
                s_runtime.alignProbeDecay = false;
                s_runtime.dqDutySign = 1;
                s_runtime.alignInitialized = true;
                printf("[AUTOTUNE] L_ALIGN current decay done: Iphase=%.3f A, "
                       "probing dq polarity, duty=%.6f..%.6f\n",
                       AutoTune_GetPhaseCurrentPeakA(),
                       AUTOTUNE_ALIGN_PROBE_DUTY,
                       AUTOTUNE_ALIGN_PROBE_MAX_DUTY);
            }

            if (!s_runtime.alignPolarityValid)
            {
                if (s_runtime.alignProbeTick > 0U &&
                    fabsf(id) >= AUTOTUNE_ALIGN_PROBE_MIN_CURRENT_A)
                {
                    int8_t currentSign = (id >= 0.0f) ? 1 : -1;
                    if (currentSign == s_runtime.alignProbeSignCandidate)
                    {
                        if (s_runtime.alignProbeSignCount < UINT16_MAX)
                        {
                            s_runtime.alignProbeSignCount++;
                        }
                    }
                    else
                    {
                        s_runtime.alignProbeSignCandidate = currentSign;
                        s_runtime.alignProbeSignCount = 1U;
                    }

                    if (s_runtime.alignProbeSignCount >=
                        AUTOTUNE_ALIGN_PROBE_SIGN_TICKS)
                    {
                        s_runtime.dqDutySign = currentSign;
                        s_runtime.alignPolarityValid = true;
                        s_runtime.alignProbeDecay = true;
                        s_runtime.alignEntryStableCount = 0U;
                        s_runtime.stageTick = 0U;
                        SwitchOff_PWM(g_axis.pPWMCHandle);
                        printf("[AUTOTUNE] L_ALIGN polarity: probeId=%.3f A, "
                               "duty=%.6f, sign=%d, confirm=%u/%u\n",
                               id,
                               s_runtime.baseDutyD,
                               (int)s_runtime.dqDutySign,
                               (unsigned int)s_runtime.alignProbeSignCount,
                               (unsigned int)AUTOTUNE_ALIGN_PROBE_SIGN_TICKS);
                        break;
                    }
                }
                else
                {
                    s_runtime.alignProbeSignCount = 0U;
                    s_runtime.alignProbeSignCandidate = 0;
                }

                if (s_runtime.alignProbeTick >= AUTOTUNE_ALIGN_PROBE_MAX_TICKS)
                {
                    g_rs_ident.idBiasA = id;
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_BIAS_CURRENT);
                    break;
                }

                if (s_runtime.alignProbeTick > 0U)
                {
                    s_runtime.baseDutyD = AutoTune_Clamp(
                        s_runtime.baseDutyD + AUTOTUNE_ALIGN_PROBE_DUTY_STEP,
                        AUTOTUNE_ALIGN_PROBE_DUTY,
                        AUTOTUNE_ALIGN_PROBE_MAX_DUTY);
                }
                AutoTune_ApplyDutyDQ(s_runtime.baseDutyD, 0.0f);
                s_runtime.alignProbeTick++;
                break;
            }

            if (s_runtime.alignProbeDecay)
            {
                SwitchOff_PWM(g_axis.pPWMCHandle);
                s_runtime.stageTick++;
                if (AutoTune_GetPhaseCurrentPeakA() <=
                    AUTOTUNE_ALIGN_ENTRY_CURRENT_A)
                {
                    if (s_runtime.alignEntryStableCount < UINT16_MAX)
                    {
                        s_runtime.alignEntryStableCount++;
                    }
                }
                else
                {
                    s_runtime.alignEntryStableCount = 0U;
                }

                if (s_runtime.alignEntryStableCount >=
                    AUTOTUNE_ALIGN_ENTRY_STABLE_TICKS)
                {
                    float modelDuty = (vbus > 1.0f) ?
                        (2.0f * g_rs_ident.Rs * g_rs_ident.idTargetA / vbus) : 0.005f;

                    if (modelDuty > s_runtime.baseDutyD)
                    {
                        s_runtime.baseDutyD = modelDuty;
                    }
                    s_runtime.baseDutyD = AutoTune_Clamp(
                        s_runtime.baseDutyD,
                        AUTOTUNE_ALIGN_PROBE_DUTY,
                        AUTOTUNE_ALIGN_MAX_DUTY);
                    s_runtime.alignProbeDecay = false;
                    s_runtime.alignCurrentSum = 0.0f;
                    s_runtime.alignCurrentCount = 0U;
                    s_runtime.alignStableCount = 0U;
                    s_runtime.stageTick = 0U;
                    AutoTune_ResetNoiseCalibration();
                    printf("[AUTOTUNE] L_ALIGN probe decay done: "
                           "Iphase=%.3f A, baseDuty=%.6f\n",
                           AutoTune_GetPhaseCurrentPeakA(),
                           s_runtime.baseDutyD);
                    break;
                }

                if (s_runtime.stageTick >= AUTOTUNE_ALIGN_ENTRY_MAX_TICKS)
                {
                    g_rs_ident.idBiasA = id;
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_BIAS_CURRENT);
                }
                break;
            }

            if (fabsf(id) > (g_rs_ident.idTargetA + AUTOTUNE_MAX_INJECT_CURRENT_A) ||
                fabsf(iq) > AUTOTUNE_MAX_INJECT_CURRENT_A)
            {
                AutoTune_Fail(CURR_AUTOTUNE_ERROR_OVERCURRENT);
                break;
            }

            currentError = g_rs_ident.idTargetA - id;
            s_runtime.baseDutyD += AutoTune_Clamp(
                AUTOTUNE_ALIGN_DUTY_KI * currentError,
                -AUTOTUNE_ALIGN_MAX_DUTY_STEP,
                AUTOTUNE_ALIGN_MAX_DUTY_STEP);
            s_runtime.baseDutyD = AutoTune_Clamp(s_runtime.baseDutyD,
                                                  0.0f,
                                                  AUTOTUNE_ALIGN_MAX_DUTY);
            AutoTune_ApplyDutyDQ(s_runtime.baseDutyD, 0.0f);
            s_runtime.stageTick++;

            if (s_runtime.stageTick >= AUTOTUNE_ALIGN_MIN_TICKS)
            {
                if (fabsf(currentError) <= AUTOTUNE_ALIGN_CURRENT_TOL_A &&
                    fabsf(iq) <= AUTOTUNE_ALIGN_IQ_TOL_A)
                {
                    if (s_runtime.alignStableCount < UINT16_MAX)
                    {
                        s_runtime.alignStableCount++;
                    }
                    s_runtime.alignCurrentSum += id;
                    s_runtime.alignCurrentCount++;
                    AutoTune_RecordStationaryAngle();
                    alignmentReady =
                        (s_runtime.alignStableCount >= AUTOTUNE_ALIGN_STABLE_TICKS);
                }
                else
                {
                    s_runtime.alignStableCount = 0U;
                    s_runtime.alignCurrentSum = 0.0f;
                    s_runtime.alignCurrentCount = 0U;
                    AutoTune_ResetNoiseCalibration();
                }
            }

            if (alignmentReady)
            {
                g_rs_ident.idBiasA =
                    s_runtime.alignCurrentSum / (float)s_runtime.alignCurrentCount;
                AutoTune_UpdateEncoderNoise("align");
                AutoTune_ResetMotionDetector(g_axis.fbdk.uAngleRaw);
                AutoTune_BeginInjection(AUTOTUNE_INITIAL_HF_DUTY);
                AutoTune_ApplyInjection(true);
                g_rs_ident.state = CURR_AUTOTUNE_LD_ADAPT;
                printf("[AUTOTUNE] align done: Id=%.4f A, Iq=%.4f A, "
                       "baseDuty=%.5f, raw=%lu, offsetDelta=%lu\n",
                       g_rs_ident.idBiasA,
                       iq,
                       s_runtime.baseDutyD,
                       (unsigned long)s_runtime.lockRaw,
                       (unsigned long)AutoTune_RawAngleDifference(
                           s_runtime.lockRaw,
                           (uint32_t)g_axis.posCtrl.uOffsetAngleRaw));
                printf("[AUTOTUNE] stage=LD_ADAPT\n");
            }
            else if (s_runtime.stageTick >=
                     (AUTOTUNE_ALIGN_MAX_TICKS +
                      AUTOTUNE_ALIGN_STABLE_TICKS))
            {
                g_rs_ident.idBiasA = id;
                AutoTune_Fail(CURR_AUTOTUNE_ERROR_BIAS_CURRENT);
            }
            break;
        }

        /* LD_ADAPT：逐周期调节幅值，Ipp 连续稳定后才进入正式测量。 */
        case CURR_AUTOTUNE_LD_ADAPT:
            {
                bool cycleDone = AutoTune_RunInjection(true, false);
                if (cycleDone &&
                    (s_runtime.completedCycles == 1U ||
                     s_runtime.completedCycles == 4U ||
                     s_runtime.adaptStableCount >= AUTOTUNE_ADAPT_STABLE_CYCLES ||
                     s_runtime.completedCycles == AUTOTUNE_ADAPT_MAX_CYCLES))
                {
                    printf("[AUTOTUNE] LD adapt: %u/%u, stable=%u/%u, "
                           "duty=%.6f, Ipp=%.4f A\n",
                           s_runtime.completedCycles,
                           AUTOTUNE_ADAPT_MAX_CYCLES,
                           s_runtime.adaptStableCount,
                           AUTOTUNE_ADAPT_STABLE_CYCLES,
                           s_runtime.hfDuty,
                           s_runtime.lastCurrentPeakToPeak);
                }
                if (cycleDone &&
                    s_runtime.adaptStableCount >= AUTOTUNE_ADAPT_STABLE_CYCLES)
                {
                    g_rs_ident.ldDutyAmplitude = s_runtime.hfDuty;
                    AutoTune_BeginInjection(s_runtime.hfDuty);
                    AutoTune_ApplyInjection(true);
                    g_rs_ident.state = CURR_AUTOTUNE_LD_MEASURE;
                    printf("[AUTOTUNE] stage=LD_MEASURE, duty=%.5f\n",
                           g_rs_ident.ldDutyAmplitude);
                }
                else if (cycleDone &&
                         s_runtime.completedCycles >= AUTOTUNE_ADAPT_MAX_CYCLES)
                {
                    s_lRejectReason = AUTOTUNE_L_REJECT_ADAPT_RANGE;
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_LD_INVALID);
                }
            }
            break;

        /* LD_MEASURE：使用自适应后的幅值，正式累计 Ld 半周期样本。 */
        case CURR_AUTOTUNE_LD_MEASURE:
            {
                bool cycleDone = AutoTune_RunInjection(true, true);
                if (cycleDone && (s_runtime.completedCycles % 8U) == 0U)
                {
                    printf("[AUTOTUNE] LD measure: %u/%u, samples=%u, "
                           "motionReject=%u, drift=%lu/%lu raw, Ipp=%.4f A\n",
                           s_runtime.completedCycles,
                           AUTOTUNE_MEASURE_CYCLES,
                           s_runtime.estimateCount,
                           s_runtime.motionRejectedCount,
                           (unsigned long)s_runtime.maxObservedSegmentDriftRaw,
                           (unsigned long)AutoTune_SegmentDriftRaw(
                               AUTOTUNE_LD_SEGMENT_DRIFT_DEG),
                           s_runtime.lastCurrentPeakToPeak);
                }
                if (cycleDone &&
                    s_runtime.completedCycles >= AUTOTUNE_MEASURE_CYCLES &&
                    AutoTune_HasMinimumInductanceSamples())
                {
                    if (!AutoTune_CalculateInductance(&g_rs_ident.Ld))
                    {
                        AutoTune_Fail(CURR_AUTOTUNE_ERROR_LD_INVALID);
                        break;
                    }
                    printf("[AUTOTUNE] Ld done: %.3f uH, pos=%.3f uH, neg=%.3f uH\n",
                           g_rs_ident.Ld * 1000000.0f,
                           s_runtime.positiveEstimateSum * 1000000.0f /
                               (float)s_runtime.positiveEstimateCount,
                           s_runtime.negativeEstimateSum * 1000000.0f /
                               (float)s_runtime.negativeEstimateCount);
                    s_runtime.stageTick = 0U;
                    AutoTune_ResetNoiseCalibration();
                    AutoTune_ApplyDutyDQ(s_runtime.baseDutyD, 0.0f);
                    g_rs_ident.state = CURR_AUTOTUNE_LQ_SETTLE;
                    printf("[AUTOTUNE] stage=LQ_SETTLE\n");
                }
                else if (cycleDone &&
                         s_runtime.completedCycles >= AUTOTUNE_MEASURE_MAX_CYCLES)
                {
                    s_lRejectReason = AUTOTUNE_L_REJECT_SAMPLE_COUNT;
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_LD_INVALID);
                }
            }
            break;

        /* LQ_SETTLE：停止高频注入，仅保持 d 轴偏置，等待工作点恢复稳定。 */
        case CURR_AUTOTUNE_LQ_SETTLE:
            if (!AutoTune_CheckProtection())
            {
                break;
            }
            AutoTune_ApplyDutyDQ(s_runtime.baseDutyD, 0.0f);
            AutoTune_RecordStationaryAngle();
            s_runtime.stageTick++;
            if (s_runtime.stageTick >= AUTOTUNE_LQ_SETTLE_TICKS)
            {
                AutoTune_UpdateEncoderNoise("Lq settle");
                AutoTune_BeginInjection(AUTOTUNE_INITIAL_HF_DUTY);
                AutoTune_ApplyInjection(false);
                g_rs_ident.state = CURR_AUTOTUNE_LQ_ADAPT;
                printf("[AUTOTUNE] stage=LQ_ADAPT, initialDuty=%.6f\n",
                       AUTOTUNE_INITIAL_HF_DUTY);
            }
            break;

        /* LQ_ADAPT：与 Ld 相同，按 Ipp 自动选择 q 轴注入幅值。 */
        case CURR_AUTOTUNE_LQ_ADAPT:
            {
                bool cycleDone = AutoTune_RunInjection(false, false);
                if (cycleDone &&
                    (s_runtime.completedCycles == 1U ||
                     s_runtime.completedCycles == 4U ||
                     s_runtime.adaptStableCount >= AUTOTUNE_ADAPT_STABLE_CYCLES ||
                     s_runtime.completedCycles == AUTOTUNE_ADAPT_MAX_CYCLES))
                {
                    printf("[AUTOTUNE] LQ adapt: %u/%u, stable=%u/%u, "
                           "duty=%.6f, Ipp=%.4f A\n",
                           s_runtime.completedCycles,
                           AUTOTUNE_ADAPT_MAX_CYCLES,
                           s_runtime.adaptStableCount,
                           AUTOTUNE_ADAPT_STABLE_CYCLES,
                           s_runtime.hfDuty,
                           s_runtime.lastCurrentPeakToPeak);
                }
                if (cycleDone &&
                    s_runtime.adaptStableCount >= AUTOTUNE_ADAPT_STABLE_CYCLES)
                {
                    g_rs_ident.lqDutyAmplitude = s_runtime.hfDuty;
                    AutoTune_BeginInjection(s_runtime.hfDuty);
                    AutoTune_ApplyInjection(false);
                    g_rs_ident.state = CURR_AUTOTUNE_LQ_MEASURE;
                    printf("[AUTOTUNE] stage=LQ_MEASURE, duty=%.5f\n",
                           g_rs_ident.lqDutyAmplitude);
                }
                else if (cycleDone &&
                         s_runtime.completedCycles >= AUTOTUNE_ADAPT_MAX_CYCLES)
                {
                    s_lRejectReason = AUTOTUNE_L_REJECT_ADAPT_RANGE;
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_LQ_INVALID);
                }
            }
            break;

        /* LQ_MEASURE：在保持 Id 偏置的同时，正式累计 q 轴电感样本。 */
        case CURR_AUTOTUNE_LQ_MEASURE:
            {
                bool cycleDone = AutoTune_RunInjection(false, true);
                if (cycleDone && (s_runtime.completedCycles % 8U) == 0U)
                {
                    printf("[AUTOTUNE] LQ measure: %u/%u, samples=%u, "
                           "motionReject=%u, drift=%lu/%lu raw, Ipp=%.4f A\n",
                           s_runtime.completedCycles,
                           AUTOTUNE_MEASURE_CYCLES,
                           s_runtime.estimateCount,
                           s_runtime.motionRejectedCount,
                           (unsigned long)s_runtime.maxObservedSegmentDriftRaw,
                           (unsigned long)AutoTune_SegmentDriftRaw(
                               AUTOTUNE_LQ_SEGMENT_DRIFT_DEG),
                           s_runtime.lastCurrentPeakToPeak);
                }
                if (cycleDone &&
                    s_runtime.completedCycles >= AUTOTUNE_MEASURE_CYCLES &&
                    AutoTune_HasMinimumInductanceSamples())
                {
                    if (!AutoTune_CalculateInductance(&g_rs_ident.Lq))
                    {
                        AutoTune_Fail(CURR_AUTOTUNE_ERROR_LQ_INVALID);
                        break;
                    }
                    printf("[AUTOTUNE] Lq done: %.3f uH, pos=%.3f uH, neg=%.3f uH\n",
                           g_rs_ident.Lq * 1000000.0f,
                           s_runtime.positiveEstimateSum * 1000000.0f /
                               (float)s_runtime.positiveEstimateCount,
                           s_runtime.negativeEstimateSum * 1000000.0f /
                               (float)s_runtime.negativeEstimateCount);
                    g_rs_ident.state = CURR_AUTOTUNE_VALIDATE;
                }
                else if (cycleDone &&
                         s_runtime.completedCycles >= AUTOTUNE_MEASURE_MAX_CYCLES)
                {
                    s_lRejectReason = AUTOTUNE_L_REJECT_SAMPLE_COUNT;
                    AutoTune_Fail(CURR_AUTOTUNE_ERROR_LQ_INVALID);
                }
            }
            break;

        /* VALIDATE：把辨识结果复制到控制结构，供后续 FOC 和 PI 使用。 */
        case CURR_AUTOTUNE_VALIDATE:
            g_axis.fLs = g_rs_ident.Ld;
            g_axis.fLq = g_rs_ident.Lq;
            g_rs_ident.state = CURR_AUTOTUNE_FINISH;
            break;

        /* FINISH：撤销注入、更新当前 PID 参数，并准备清理定时器状态。 */
        case CURR_AUTOTUNE_FINISH:
            AutoTune_ApplyDutyDQ(0.0f, 0.0f);
            MC_Set_Control_Mode(CTRL_MODE_SPEED);
            MC_Reset_Control_State();

            /*
             * L_ALIGN locks the rotor to electrical angle zero.  Commit that
             * original aligned encoder position before returning to closed-loop
             * FOC.  Detector rebases only track settling and are not dq offsets.
             */
            g_axis.posCtrl.uOffsetAngleRaw = s_runtime.initialLockRaw;
            g_axis.posCtrl.uCalibCount = 0U;
            g_axis.posCtrl.iZeroAngle = g_axis.fbdk.uAngleRaw;
            g_axis.posCtrl.uCircle = 0;
            g_axis.posCtrl.bCalibFlag = true;
            AutoTune_ResetRunFeedback();

            /* Restore the original R/L margin-based current-loop tuning. */
            PIDREGDQX_CURRENT_setKpWiRLmargin_si(
                &g_axis.currCtrl.pidIdIq,
                g_axis.fRs,
                g_axis.fLq,
                50.0f);

            /* Never carry alignment/injection integral voltage into normal FOC. */
            PIDREGDQX_CURRENT_setUiD_pu(&g_axis.currCtrl.pidIdIq, FIXP30(0.0f));
            PIDREGDQX_CURRENT_setUiQ_pu(&g_axis.currCtrl.pidIdIq, FIXP30(0.0f));

            printf("[AUTOTUNE] finished: Rs=%.6f ohm, Ld=%.3f uH, "
                   "Lq=%.3f uH, Dld=%.5f, Dlq=%.5f\n",
                   g_rs_ident.Rs,
                   g_rs_ident.Ld * 1000000.0f,
                   g_rs_ident.Lq * 1000000.0f,
                   g_rs_ident.ldDutyAmplitude,
                   g_rs_ident.lqDutyAmplitude);

            printf("[AUTOTUNE] current PI: margin=50.0, Kp=%.6f V/A, "
                   "Wi=%.3f rad/s\n",
                   g_axis.currCtrl.pidIdIq.Kp,
                   PIDREGDQX_CURRENT_getWi_si(&g_axis.currCtrl.pidIdIq));
            printf("[AUTOTUNE] encoder zero committed: raw=%lu, currentRaw=%lu\n",
                   (unsigned long)g_axis.posCtrl.uOffsetAngleRaw,
                   (unsigned long)g_axis.fbdk.uAngleRaw);
            s_runtime.stageTick = 0U;
            LL_TIM_ClearFlag_UPDATE(TIM1);
            g_rs_ident.state = CURR_AUTOTUNE_RECOVER;
            break;

        /* RECOVER：避免日志输出或状态切换造成第一次 FOC 更新超时。 */
        case CURR_AUTOTUNE_RECOVER:
            /*
             * 前一状态中的 printf 可能跨越多个 PWM 周期。
             * 等待若干个完整的定时器更新后再进入 FOC，并在进入 RUN
             * 前立即清除更新标志，避免误触发 FOC 超时。
             */
            LL_TIM_ClearFlag_UPDATE(TIM1);
            s_runtime.stageTick++;
            if (s_runtime.stageTick >= AUTOTUNE_RECOVERY_TICKS)
            {
                LL_TIM_ClearFlag_UPDATE(TIM1);
                g_bStartCurrentAutoTune = false;

                /*
                 * satune 会同时置位速度辨识请求。电流辨识成功后保留刚刚
                 * 更新的 Rs/Ld/Lq 和电流环 PI，直接交给速度辨识模块；
                 * ratune 没有速度请求，因此维持原逻辑进入 RUN。
                 */
                g_axis.state = g_bStartSpeedAutoTune ?
                               AXIS_STATE_SPEED_AUTOTUNE : AXIS_STATE_RUN;
            }
            break;

        case CURR_AUTOTUNE_FAULT:
        default:
            AutoTune_Fail(CURR_AUTOTUNE_ERROR_LQ_INVALID);
            break;
    }
}
