#include "param_identify.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "motor_parameters.h"
#include "mc_interface.h"
#include "curr_fbdk.h"
#include "foc.h"
#include "speed_pos_fbdk.h"
#include "curr_autotune.h"
#include "speed_autotune.h"
#include "stm32g4xx_hal_flash.h"
#include "pidregdqx_current.h"

static ParamIdHandle_t s_param_id_module;
static bool s_param_id_flash_save_pending = false;
static ParamIdFlashData_t s_param_id_flash_pending_data;
static uint8_t s_can_node_id = 1U;
static ParamIdDebugData_t s_param_id_debug_data;

#define PARAM_ID_RS_MIN_OHM          (0.0001f)
#define PARAM_ID_RS_MAX_OHM          (5.0f)
#define PARAM_ID_L_MIN_H             (1.0e-7f)
#define PARAM_ID_L_MAX_H             (5.0e-3f)
/*
 * Low-inductance motors (~tens of uH) need stronger excitation and a shorter
 * measurement window, otherwise ADC offset/deadtime dominate the slope fit.
 */
#define PARAM_ID_L_STEP_DUTY         (0.020f)
#define PARAM_ID_L_MIN_STEP_DUTY     (0.010f)
#define PARAM_ID_L_MAX_STEP_DUTY     (0.120f)
#define PARAM_ID_L_FINAL_AVG_DIV     (4U)
#define PARAM_ID_L_TRACE_MAX         (1024U)

/* ---------- Local helpers ---------- */
static float ParamId_AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static bool ParamId_IsBusy(const ParamIdHandle_t *h)
{
    return (h != NULL) &&
           (h->state == PARAM_ID_STATE_PREPARE ||
            h->state == PARAM_ID_STATE_LOCK_CHECK ||
            h->state == PARAM_ID_STATE_RUN);
}

static void ParamId_ResetResult(ParamIdResult_t *r)
{
    memset(r, 0, sizeof(*r));
}

static void ParamId_ResetDebugData(void)
{
    memset(&s_param_id_debug_data, 0, sizeof(s_param_id_debug_data));
}

static bool ParamId_CurrentAutoTuneIsFinal(void)
{
    return (g_rs_ident.state == CURR_AUTOTUNE_FINISH ||
            g_rs_ident.state == CURR_AUTOTUNE_RECOVER ||
            g_rs_ident.state == CURR_AUTOTUNE_FAULT);
}

static bool ParamId_SpeedAutoTuneIsFinal(void)
{
    return (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FINISH ||
            g_speedAutoTuneResult.state == SPEED_AUTOTUNE_RECOVER ||
            g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FAULT);
}

static bool ParamId_CurrentAutoTuneHasValidResult(void)
{
    return (g_rs_ident.state == CURR_AUTOTUNE_FINISH ||
            g_rs_ident.state == CURR_AUTOTUNE_RECOVER);
}

static bool ParamId_SpeedAutoTuneHasValidResult(void)
{
    return (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FINISH ||
            g_speedAutoTuneResult.state == SPEED_AUTOTUNE_RECOVER);
}

static void ParamId_SyncResultFromSG(ParamIdResult_t *result)
{
    if (result == NULL)
    {
        return;
    }

    memset(result, 0, sizeof(*result));

    if (g_rs_ident.Rs > 0.0f)
    {
        result->validRs = ParamId_CurrentAutoTuneHasValidResult() || g_bStartCurrentAutoTune;
        result->rs_ohm = g_rs_ident.Rs;
    }

    if (g_rs_ident.Ld > 0.0f)
    {
        result->validLd = ParamId_CurrentAutoTuneHasValidResult() || g_bStartCurrentAutoTune;
        result->ld_h = g_rs_ident.Ld;
    }

    if (g_rs_ident.Lq > 0.0f)
    {
        result->validLq = ParamId_CurrentAutoTuneHasValidResult() || g_bStartCurrentAutoTune;
        result->lq_h = g_rs_ident.Lq;
    }

    /*
     * SG 当前没有单独的 Ke/J/B 持久化链路。
     * 这里保留字段，但不把它们当作当前自动整定结果的一部分。
     */
    result->validKe = false;
    result->ke_v_per_rad_s = 0.0f;
}

static void ParamId_SyncDebugFromSG(void)
{
    s_param_id_debug_data.calc_id_a = FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE;
    s_param_id_debug_data.calc_iq_a = FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE;

    if (g_bStartCurrentAutoTune)
    {
        s_param_id_debug_data.i_avg_a = g_rs_ident.Rs;
        s_param_id_debug_data.v_avg_v = g_rs_ident.ldDutyAmplitude;
    }
    else if (g_bStartSpeedAutoTune)
    {
        s_param_id_debug_data.i_avg_a = g_speedAutoTuneResult.kp_A_per_eHz;
        s_param_id_debug_data.v_avg_v = g_speedAutoTuneResult.ki_A_per_eHz_s;
    }
    else
    {
        s_param_id_debug_data.i_avg_a = 0.0f;
        s_param_id_debug_data.v_avg_v = 0.0f;
    }
}

static void ParamId_ForceSafeOutput(void)
{
    Duty_Ddq_t duty = {0};
    duty.D = FIXP30(0.0f);
    duty.Q = FIXP30(0.0f);
    MC_Set_Duty_Cycle(duty);
}

static float ParamId_GetIqA(void)
{
    return FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE;
}

static float ParamId_GetIdA(void)
{
    return FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE;
}

static float ParamId_GetVbusV(void)
{
    return FIXP30_toF(g_axis.busVoltage) * VOLTAGE_SCALE;
}

static float ParamId_GetSpeedRpm(void)
{
    /* speedMeas_pu is electrical Hz / FREQUENCY_SCALE */
    float fe_hz = FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) * FREQUENCY_SCALE;
    return (60.0f * fe_hz) / (float)MC_Get_Pole_Pairs();
}

static float ParamId_GetElecHz(void)
{
    return FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) * FREQUENCY_SCALE;
}

static float ParamId_GetElecSpeedRadPs(void)
{
    return 2.0f * 3.1415926f * ParamId_GetElecHz();
}

static void ParamId_GetPhaseCurrentsA(float *ir, float *is, float *it)
{
    if (ir != NULL)
    {
        *ir = FIXP30_toF(g_axis.currCtrl.IrstMeas.R) * CURRENT_SCALE;
    }

    if (is != NULL)
    {
        *is = FIXP30_toF(g_axis.currCtrl.IrstMeas.S) * CURRENT_SCALE;
    }

    if (it != NULL)
    {
        *it = FIXP30_toF(g_axis.currCtrl.IrstMeas.T) * CURRENT_SCALE;
    }
}

static void ParamId_GetAppliedPhaseVoltages(float *vr, float *vs, float *vt)
{
    float vbusV = ParamId_GetVbusV();

    if (g_axis.pPWMCHandle == NULL)
    {
        if (vr != NULL) { *vr = 0.0f; }
        if (vs != NULL) { *vs = 0.0f; }
        if (vt != NULL) { *vt = 0.0f; }
        return;
    }

    if (vr != NULL)
    {
        *vr = FIXP30_toF(g_axis.pPWMCHandle->drstOut_pu.R) * (0.5f * vbusV);
    }

    if (vs != NULL)
    {
        *vs = FIXP30_toF(g_axis.pPWMCHandle->drstOut_pu.S) * (0.5f * vbusV);
    }

    if (vt != NULL)
    {
        *vt = FIXP30_toF(g_axis.pPWMCHandle->drstOut_pu.T) * (0.5f * vbusV);
    }
}

static void ParamId_GetAppliedVoltageDQ(float *vd, float *vq)
{
    Duty_Drst_t appliedDrst;
    Currents_Irst_t appliedIrst;
    Currents_Iab_t appliedIab;
    Currents_Idq_t appliedIdq;
    fixp30_t anglePark_pu;
    FIXP_CosSin_t cossinPark;
    float vbusV = ParamId_GetVbusV();

    if ((vd == NULL) || (vq == NULL))
    {
        return;
    }

    *vd = 0.0f;
    *vq = 0.0f;

    if (g_axis.pPWMCHandle == NULL)
    {
        return;
    }

    appliedDrst = g_axis.pPWMCHandle->drstOut_pu;
    appliedIrst.R = appliedDrst.R;
    appliedIrst.S = appliedDrst.S;
    appliedIrst.T = appliedDrst.T;

    Get_Angle(&anglePark_pu);
    FIXP30_CosSinPU(anglePark_pu, &cossinPark);
    Clarke_Current(appliedIrst, &appliedIab);
    Park_Current(appliedIab, &cossinPark, &appliedIdq);

    /*
     * drstOut_pu is centered around 50% duty. The corresponding phase voltage
     * relative to the half-bus midpoint is approximately duty * Vbus / 2.
     */
    *vd = FIXP30_toF(appliedIdq.D) * (0.5f * vbusV);
    *vq = FIXP30_toF(appliedIdq.Q) * (0.5f * vbusV);
}

static float ParamId_GetAppliedVoltageAxis(bool isD)
{
    float vd;
    float vq;

    ParamId_GetAppliedVoltageDQ(&vd, &vq);
    return isD ? vd : vq;
}

static bool ParamId_CheckProtection(const ParamIdHandle_t *h)
{
    float iqAbs = ParamId_AbsF(ParamId_GetIqA());
    float idAbs = ParamId_AbsF(ParamId_GetIdA());
    float speedAbsRpm = ParamId_AbsF(ParamId_GetSpeedRpm());

    if (iqAbs > h->cfg.maxCurrentA || idAbs > h->cfg.maxCurrentA)
    {
        return false;
    }
    if (speedAbsRpm > h->cfg.maxSpeedRpm)
    {
        return false;
    }
    if (h->tick > h->cfg.maxRunTicks)
    {
        return false;
    }
    return true;
}

static ParamIdStep_t ParamId_NextStep(ParamIdStep_t step)
{
    switch (step)
    {
        case PARAM_ID_STEP_RS: return PARAM_ID_STEP_LD;
        case PARAM_ID_STEP_LD: return PARAM_ID_STEP_LQ;
        case PARAM_ID_STEP_LQ: return PARAM_ID_STEP_KE;
        default: return PARAM_ID_STEP_KE;
    }
}

/* ---------- Flash persistence ---------- */
#define PARAM_ID_FLASH_MAGIC        (0x50494431UL) /* "PID1" */
#define PARAM_ID_FLASH_VERSION      (0x00010002UL)
/* User may move this address to a dedicated page in linker script. */
#define PARAM_ID_FLASH_ADDR         (0x0803F800UL)
#define PARAM_ID_FLASH_PAGE_SIZE    (2048UL)

/* Default motor preset for boards that should run without Rs/Ld auto-identification. */
#define PARAM_ID_DEFAULT_RS_OHM     (0.120f)
#define PARAM_ID_DEFAULT_LD_H       (50.0e-6f)
#define PARAM_ID_DEFAULT_LQ_H       (50.0e-6f)
#define PARAM_ID_DEFAULT_KE         (0.0f)
#define PARAM_ID_DEFAULT_POLE_PAIRS (7U)
#define PARAM_ID_DEFAULT_CURR_KP    (0.15f)
#define PARAM_ID_DEFAULT_CURR_WI    (20.0f)

/* J/B least-square estimation runtime cache */
typedef struct
{
    bool inited;
    float lastWe;
    float estJ;
    float estB;
    float s11;
    float s12;
    float s22;
    float sy1;
    float sy2;
    uint32_t n;
} ParamIdJBCache_t;

static uint32_t ParamId_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i = 0U;
    while (i < len)
    {
        crc ^= (uint32_t)data[i++];
        for (uint32_t b = 0U; b < 8U; b++)
        {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1UL));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static bool ParamId_IsLockRequiredStep(ParamIdStep_t step)
{
    return (step == PARAM_ID_STEP_RS || step == PARAM_ID_STEP_LD || step == PARAM_ID_STEP_LQ);
}

static bool ParamId_IsValidCanNodeId(uint8_t nodeId)
{
    return (nodeId <= CAN_NODE_ID_MASK);
}

static uint32_t ParamId_AngleRawDiffNative(uint32_t a, uint32_t b, uint32_t counts)
{
    uint32_t d;
    uint32_t rev;

    if (counts <= 1U)
    {
        return 0U;
    }

    a %= counts;
    b %= counts;
    d = (a >= b) ? (a - b) : (counts - b + a);
    rev = counts - d;
    return (d < rev) ? d : rev;
}

static uint32_t ParamId_CompatDeltaToNative(uint16_t compatDelta, uint32_t nativeCounts)
{
    if (nativeCounts <= 1U)
    {
        return 0U;
    }

    return (uint32_t)(((uint64_t)compatDelta * (uint64_t)(nativeCounts - 1U)) / 65535ULL);
}

static bool ParamId_FlashWritePage(uint32_t addr, const void *data, uint32_t bytes)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t pageError = 0U;
    const uint64_t *src = (const uint64_t *)data;
    uint32_t words64 = (uint32_t)((bytes + 7U) / 8U);

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (addr - FLASH_BASE) / PARAM_ID_FLASH_PAGE_SIZE;
    erase.NbPages = 1U;
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return false;
    }

    for (uint32_t i = 0U; i < words64; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i * 8U, src[i]) != HAL_OK)
        {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    (void)HAL_FLASH_Lock();
    return true;
}

static bool ParamId_FlashErasePage(uint32_t addr)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t pageError = 0U;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = (addr - FLASH_BASE) / PARAM_ID_FLASH_PAGE_SIZE;
    erase.NbPages = 1U;
    if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK)
    {
        (void)HAL_FLASH_Lock();
        return false;
    }

    (void)HAL_FLASH_Lock();
    return true;
}

/*
 * Apply current-loop PI gains from identified R/L parameter.
 * Priority: Lq -> average(Ld,Lq) -> Ld.
 */
static void ParamId_ApplyCurrentPiFromResult(const ParamIdResult_t *r)
{
    const float margin = 5.0f;
    float L = 0.0f;
    if (r == NULL || !r->validRs)
    {
        return;
    }
    if (r->validLd && r->validLq)
    {
        L = 0.5f * (r->ld_h + r->lq_h);
    }
    else if (r->validLq)
    {
        L = r->lq_h;
    }
    else if (r->validLd)
    {
        L = r->ld_h;
    }

    if (L > 1e-7f)
    {
        PIDREGDQX_CURRENT_setKpWiRLmargin_si(&g_axis.currCtrl.pid_IdIqX_obj, r->rs_ohm, L, margin);
    }
}

/* Reset J/B estimator before KE run. */
static void ParamId_JBReset(ParamIdJBCache_t *jb)
{
    memset(jb, 0, sizeof(*jb));
}

/* Accumulate one sample for Te = J*alpha + B*omega least-square fit. */
static void ParamId_JBPush(ParamIdJBCache_t *jb, float we, float te, float dt)
{
    if (!jb->inited)
    {
        jb->lastWe = we;
        jb->inited = true;
        return;
    }

    float alpha = (we - jb->lastWe) / dt;
    jb->lastWe = we;

    jb->s11 += alpha * alpha;
    jb->s12 += alpha * we;
    jb->s22 += we * we;
    jb->sy1 += alpha * te;
    jb->sy2 += we * te;
    jb->n++;
}

/* Solve 2x2 normal equation for J and B. */
static void ParamId_JBSolve(ParamIdJBCache_t *jb)
{
    float det = jb->s11 * jb->s22 - jb->s12 * jb->s12;
    if (jb->n < 20U || ParamId_AbsF(det) < 1e-9f)
    {
        return;
    }

    jb->estJ = (jb->sy1 * jb->s22 - jb->sy2 * jb->s12) / det;
    jb->estB = (jb->s11 * jb->sy2 - jb->s12 * jb->sy1) / det;
}

/* Build persist payload from current runtime identified values. */
static void ParamId_BuildFlashData(ParamIdFlashData_t *out, const ParamIdResult_t *result, const ParamIdJBCache_t *jb)
{
    if (out == NULL || result == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->result = *result;
    out->pole_pairs = (uint32_t)MC_Get_Pole_Pairs();
    out->can_node_id = s_can_node_id;
    out->curr_kp_si = PIDREGDQX_CURRENT_getKp_si(&g_axis.currCtrl.pid_IdIqX_obj);
    out->curr_wi_si = PIDREGDQX_CURRENT_getWi_si(&g_axis.currCtrl.pid_IdIqX_obj);
    if (jb != NULL)
    {
        out->mech_j = jb->estJ;
        out->mech_b = jb->estB;
    }
}

static void ParamId_CopyResultToAxis(const ParamIdResult_t *result)
{
    if (result == NULL)
    {
        return;
    }

    if (result->validRs)
    {
        g_axis.fRs = result->rs_ohm;
    }

    if (result->validLq)
    {
        g_axis.fLs = result->lq_h;
    }
    else if (result->validLd)
    {
        g_axis.fLs = result->ld_h;
    }

    if (result->validKe)
    {
        g_axis.fKt = result->ke_v_per_rad_s;
    }
}

static void ParamId_BuildDefaultFlashData(ParamIdFlashData_t *out)
{
    if (out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->result.validRs = true;
    out->result.validLd = true;
    out->result.validLq = true;
    out->result.validKe = false;
    out->result.rs_ohm = PARAM_ID_DEFAULT_RS_OHM;
    out->result.ld_h = PARAM_ID_DEFAULT_LD_H;
    out->result.lq_h = PARAM_ID_DEFAULT_LQ_H;
    out->result.ke_v_per_rad_s = PARAM_ID_DEFAULT_KE;
    out->pole_pairs = PARAM_ID_DEFAULT_POLE_PAIRS;
    out->can_node_id = 1U;
    out->curr_kp_si = PARAM_ID_DEFAULT_CURR_KP;
    out->curr_wi_si = PARAM_ID_DEFAULT_CURR_WI;
}

/* ---------- Public API ---------- */
void ParamId_GetDefaultConfig(ParamIdConfig_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->maxCurrentA = 6.0f;
    cfg->maxSpeedRpm = 800.0f;
    cfg->maxRunTicks = 15000U;

    cfg->rsCurrentA = 1.5f;
    cfg->ldStepCurrentA = 1.0f;
    cfg->lqStepCurrentA = 1.0f;
    cfg->keTargetSpeedRpm = 200.0f;

    cfg->settleTicks = 300U;
    cfg->sampleTicks = 400U;
    cfg->lockCheckTicks = 300U;
    cfg->lockMaxAngleDeltaRaw = 80U;
    cfg->lockMaxSpeedRpm = 5.0f;
}

void ParamId_Init(ParamIdHandle_t *h)
{
    if (h == NULL)
    {
        return;
    }

    memset(h, 0, sizeof(*h));
    ParamId_GetDefaultConfig(&h->cfg);
    h->state = PARAM_ID_STATE_IDLE;
    h->requestedStep = PARAM_ID_STEP_ALL;
    h->activeStep = PARAM_ID_STEP_RS;
    ParamId_SyncResultFromSG(&h->result);
}

ParamIdRet_t ParamId_SetConfig(ParamIdHandle_t *h, const ParamIdConfig_t *cfg)
{
    if (h == NULL || cfg == NULL)
    {
        return PARAM_ID_ERR_BAD_ARG;
    }
    if (ParamId_IsBusy(h))
    {
        return PARAM_ID_ERR_BUSY;
    }
    h->cfg = *cfg;
    return PARAM_ID_OK;
}

ParamIdRet_t ParamId_Start(ParamIdHandle_t *h, ParamIdStep_t step)
{
    if (h == NULL)
    {
        return PARAM_ID_ERR_BAD_ARG;
    }
    if (ParamId_IsBusy(h))
    {
        return PARAM_ID_ERR_BUSY;
    }
    if (step > PARAM_ID_STEP_ALL)
    {
        return PARAM_ID_ERR_BAD_ARG;
    }

    ParamId_ResetResult(&h->result);
    ParamId_ResetDebugData();
    h->requestedStep = step;
    h->activeStep = (step == PARAM_ID_STEP_ALL) ? PARAM_ID_STEP_RS : step;
    h->tick = 0U;
    h->subTick = 0U;
    h->stopRequested = false;
    h->state = PARAM_ID_STATE_PREPARE;
    return PARAM_ID_OK;
}

ParamIdRet_t ParamId_Stop(ParamIdHandle_t *h)
{
    if (h == NULL)
    {
        return PARAM_ID_ERR_BAD_ARG;
    }
    if (!ParamId_IsBusy(h))
    {
        return PARAM_ID_ERR_NOT_RUNNING;
    }
    h->stopRequested = true;
    return PARAM_ID_OK;
}

ParamIdState_t ParamId_GetState(const ParamIdHandle_t *h)
{
    if (h == NULL)
    {
        return PARAM_ID_STATE_FAULT;
    }
    return h->state;
}

const ParamIdResult_t *ParamId_GetResult(const ParamIdHandle_t *h)
{
    if (h == NULL)
    {
        return NULL;
    }
    if (h == &s_param_id_module)
    {
        ParamId_SyncResultFromSG(&s_param_id_module.result);
        return &s_param_id_module.result;
    }
    return &h->result;
}

void ParamId_GetDebugData(ParamIdDebugData_t *data)
{
    if (data == NULL)
    {
        return;
    }

    ParamId_SyncDebugFromSG();
    *data = s_param_id_debug_data;
}

/* ---------- Core state machine ---------- */
void ParamId_Service(ParamIdHandle_t *h)
{
    /*
     * 0512 当前不再运行独立的旧参数辨识状态机。
     * 结果由 SG 风格的 CurrAutoTune / SpeedAutoTune 直接产出，
     * 这里只保留接口兼容，不再驱动旧的 step-response 计算链路。
     */
    if (h == NULL)
    {
        return;
    }

    ParamId_SyncResultFromSG(&h->result);
    ParamId_SyncDebugFromSG();

    if (g_bStartCurrentAutoTune)
    {
        h->state = PARAM_ID_STATE_RUN;
    }
    else if (g_bStartSpeedAutoTune)
    {
        h->state = PARAM_ID_STATE_RUN;
    }
    else if (g_rs_ident.state == CURR_AUTOTUNE_FAULT ||
             g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FAULT)
    {
        h->state = PARAM_ID_STATE_FAULT;
    }
    else if (ParamId_CurrentAutoTuneHasValidResult() || ParamId_SpeedAutoTuneHasValidResult())
    {
        h->state = PARAM_ID_STATE_DONE;
    }
    else
    {
        h->state = PARAM_ID_STATE_IDLE;
    }
}

static void ParamId_LegacyServiceUnused(ParamIdHandle_t *h)
{
    (void)h;
}

bool ParamId_SaveToFlash(const ParamIdFlashData_t *data)
{
    (void)data;
    return false;
}

bool ParamId_LoadFromFlash(ParamIdFlashData_t *data)
{
    (void)data;
    return false;
}

bool ParamId_ClearFlash(void)
{
    s_param_id_flash_save_pending = false;
    memset(&s_param_id_flash_pending_data, 0, sizeof(s_param_id_flash_pending_data));
    return false;
}

bool ParamId_ApplyFlashDataToAxis(const ParamIdFlashData_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    s_param_id_module.result = data->result;

    if (data->pole_pairs != 0U)
    {
        g_axis.uPolePairs = (uint8_t)data->pole_pairs;
    }

    if (ParamId_IsValidCanNodeId(data->can_node_id))
    {
        (void)ParamId_SetCanNodeId(data->can_node_id);
    }

    ParamId_CopyResultToAxis(&data->result);

    if (data->curr_kp_si > 0.0f)
    {
        PIDREGDQX_CURRENT_setKp_si(&g_axis.currCtrl.pid_IdIqX_obj, data->curr_kp_si);
    }

    if (data->curr_wi_si > 0.0f)
    {
        PIDREGDQX_CURRENT_setWi_si(&g_axis.currCtrl.pid_IdIqX_obj, data->curr_wi_si);
    }

    return true;
}

bool ParamId_RestoreFromFlashToAxis(void)
{
    ParamIdFlashData_t data;

    if (!ParamId_LoadFromFlash(&data))
    {
        return false;
    }

    return ParamId_ApplyFlashDataToAxis(&data);
}

uint8_t ParamId_GetCanNodeId(void)
{
    return s_can_node_id;
}

bool ParamId_SetCanNodeId(uint8_t nodeId)
{
    if (!ParamId_IsValidCanNodeId(nodeId))
    {
        return false;
    }

    s_can_node_id = nodeId;
    return true;
}

bool ParamId_SaveCanNodeIdToFlash(uint8_t nodeId)
{
    if (!ParamId_IsValidCanNodeId(nodeId))
    {
        return false;
    }

    (void)ParamId_SetCanNodeId(nodeId);
    return true;
}

void ParamId_ModuleInit(void)
{
    ParamId_Init(&s_param_id_module);
    s_param_id_flash_save_pending = false;
    memset(&s_param_id_flash_pending_data, 0, sizeof(s_param_id_flash_pending_data));
    s_can_node_id = 1U;
    ParamId_ResetDebugData();
    ParamId_SyncResultFromSG(&s_param_id_module.result);
}

ParamIdRet_t ParamId_ModuleStart(ParamIdStep_t step)
{
    ParamIdRet_t ret = ParamId_Start(&s_param_id_module, step);
    ParamId_SyncResultFromSG(&s_param_id_module.result);
    return ret;
}

ParamIdRet_t ParamId_ModuleStop(void)
{
    return ParamId_Stop(&s_param_id_module);
}

void ParamId_ModuleService(void)
{
    ParamId_Service(&s_param_id_module);
}

void ParamId_ModuleBackgroundService(void)
{
    /*
     * Flash persistence is intentionally disabled for now.
     * Keep the framework and payload types in place, but do not write.
     */
    s_param_id_flash_save_pending = false;
}

ParamIdState_t ParamId_ModuleGetState(void)
{
    if (g_bStartCurrentAutoTune)
    {
        if (g_rs_ident.state == CURR_AUTOTUNE_FAULT)
        {
            return PARAM_ID_STATE_FAULT;
        }
        if (g_rs_ident.state == CURR_AUTOTUNE_FINISH || g_rs_ident.state == CURR_AUTOTUNE_RECOVER)
        {
            return PARAM_ID_STATE_DONE;
        }
        return PARAM_ID_STATE_RUN;
    }

    if (g_bStartSpeedAutoTune)
    {
        if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FAULT)
        {
            return PARAM_ID_STATE_FAULT;
        }
        if (g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FINISH || g_speedAutoTuneResult.state == SPEED_AUTOTUNE_RECOVER)
        {
            return PARAM_ID_STATE_DONE;
        }
        return PARAM_ID_STATE_RUN;
    }

    if (ParamId_CurrentAutoTuneIsFinal() || ParamId_SpeedAutoTuneIsFinal())
    {
        if (g_rs_ident.state == CURR_AUTOTUNE_FAULT ||
            g_speedAutoTuneResult.state == SPEED_AUTOTUNE_FAULT)
        {
            return PARAM_ID_STATE_FAULT;
        }
        return PARAM_ID_STATE_DONE;
    }

    return PARAM_ID_STATE_IDLE;
}

const ParamIdResult_t *ParamId_ModuleGetResult(void)
{
    ParamId_SyncResultFromSG(&s_param_id_module.result);
    return ParamId_GetResult(&s_param_id_module);
}
