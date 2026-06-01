#include "param_identify.h"

#include <math.h>
#include <string.h>

#include "main.h"
#include "motor_parameters.h"
#include "mc_interface.h"
#include "curr_fbdk.h"
#include "speed_pos_fbdk.h"
#include "stm32g4xx_hal_flash.h"
#include "pidregdqx_current.h"

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
    return (60.0f * fe_hz) / (float)POLE_PAIR_NUM;
}

static fixp30_t ParamId_GetIqPu(void)
{
    return g_axis.currCtrl.calcIdq.Q;
}

static fixp30_t ParamId_GetIdPu(void)
{
    return g_axis.currCtrl.calcIdq.D;
}

static fixp30_t ParamId_GetSpeedPu(void)
{
    return g_axis.speedCtrl.speedMeas_pu;
}

static fixp30_t ParamId_GetVbusPu(void)
{
    return g_axis.busVoltage;
}

static bool ParamId_CheckProtection(const ParamIdHandle_t *h)
{
    fixp30_t iqAbs_pu = FIXP30_abs(ParamId_GetIqPu());
    fixp30_t idAbs_pu = FIXP30_abs(ParamId_GetIdPu());
    fixp30_t speedAbs_pu = FIXP30_abs(ParamId_GetSpeedPu());
    fixp30_t iLimit_pu = FIXP30(h->cfg.maxCurrentA / CURRENT_SCALE);
    float fElecHzLimit = (h->cfg.maxSpeedRpm * (float)POLE_PAIR_NUM) / 60.0f;
    fixp30_t speedLimit_pu = FIXP30(fElecHzLimit / FREQUENCY_SCALE);

    if (iqAbs_pu > iLimit_pu || idAbs_pu > iLimit_pu)
    {
        return false;
    }
    if (speedAbs_pu > speedLimit_pu)
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
#define PARAM_ID_FLASH_VERSION      (0x00010000UL)
/* User may move this address to a dedicated page in linker script. */
#define PARAM_ID_FLASH_ADDR         (0x0803F800UL)
#define PARAM_ID_FLASH_PAGE_SIZE    (2048UL)

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

static uint16_t ParamId_AngleRawDiff(uint16_t a, uint16_t b)
{
    uint16_t d = (a >= b) ? (a - b) : (uint16_t)(65535U - b + a + 1U);
    uint16_t rev = (uint16_t)(65535U - d + 1U);
    return (d < rev) ? d : rev;
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
    if (r->validLq)
    {
        L = r->lq_h;
    }
    else if (r->validLd && r->validLq)
    {
        L = 0.5f * (r->ld_h + r->lq_h);
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
    out->curr_kp_si = PIDREGDQX_CURRENT_getKp_si(&g_axis.currCtrl.pid_IdIqX_obj);
    out->curr_wi_si = PIDREGDQX_CURRENT_getWi_si(&g_axis.currCtrl.pid_IdIqX_obj);
    if (jb != NULL)
    {
        out->mech_j = jb->estJ;
        out->mech_b = jb->estB;
    }
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
    return &h->result;
}

/* ---------- Core state machine ---------- */
void ParamId_Service(ParamIdHandle_t *h)
{
    static float sumA = 0.0f;
    static float sumB = 0.0f;
    static float lastCurrA = 0.0f;
    static int64_t sumIq_pu = 0;
    static int64_t sumVq_pu = 0;
    static uint16_t lockStartRaw = 0U;
    static ParamIdJBCache_t jbCache;

    if (h == NULL)
    {
        return;
    }

    if (h->state == PARAM_ID_STATE_IDLE || h->state == PARAM_ID_STATE_DONE || h->state == PARAM_ID_STATE_FAULT)
    {
        return;
    }

    h->tick++;
    h->subTick++;

    if (h->stopRequested)
    {
        ParamId_ForceSafeOutput();
        h->state = PARAM_ID_STATE_IDLE;
        return;
    }

    if (!ParamId_CheckProtection(h))
    {
        ParamId_ForceSafeOutput();
        h->state = PARAM_ID_STATE_FAULT;
        return;
    }

    if (h->state == PARAM_ID_STATE_PREPARE)
    {
        sumA = 0.0f;
        sumB = 0.0f;
        lastCurrA = 0.0f;
        sumIq_pu = 0;
        sumVq_pu = 0;

        MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
        MC_Set_Speed_Reference(0.0f);

        Duty_Ddq_t duty = {0};
        duty.D = FIXP30(0.0f);
        duty.Q = FIXP30(0.0f);
        MC_Set_Duty_Cycle(duty);

        h->subTick = 0U;
        if (ParamId_IsLockRequiredStep(h->activeStep))
        {
            lockStartRaw = Get_Angle_Raw();
            h->state = PARAM_ID_STATE_LOCK_CHECK;
        }
        else
        {
            if (h->activeStep == PARAM_ID_STEP_KE)
            {
                ParamId_JBReset(&jbCache);
            }
            h->state = PARAM_ID_STATE_RUN;
        }
        return;
    }

    if (h->state == PARAM_ID_STATE_LOCK_CHECK)
    {
        /*
         * Rotor lock verification for static tests (Rs/Ld/Lq):
         * 1) Angle raw drift must stay within threshold.
         * 2) Speed must stay near zero.
         * Both must hold for lockCheckTicks.
         */
        uint16_t nowRaw = Get_Angle_Raw();
        uint16_t rawDiff = ParamId_AngleRawDiff(nowRaw, lockStartRaw);
        fixp30_t speedAbs_pu = FIXP30_abs(ParamId_GetSpeedPu());
        float fElecHzLock = (h->cfg.lockMaxSpeedRpm * (float)POLE_PAIR_NUM) / 60.0f;
        fixp30_t speedLockLimit_pu = FIXP30(fElecHzLock / FREQUENCY_SCALE);
        if (rawDiff > h->cfg.lockMaxAngleDeltaRaw || speedAbs_pu > speedLockLimit_pu)
        {
            ParamId_ForceSafeOutput();
            h->state = PARAM_ID_STATE_FAULT;
            return;
        }
        if (h->subTick >= h->cfg.lockCheckTicks)
        {
            h->subTick = 0U;
            h->state = PARAM_ID_STATE_RUN;
        }
        return;
    }

    /* Main run branch */
    switch (h->activeStep)
    {
        case PARAM_ID_STEP_RS:
        {
            /* Lock rotor and inject small q-axis duty, then estimate Rs from V/I. */
            float targetA = h->cfg.rsCurrentA;
            fixp30_t iq_pu = ParamId_GetIqPu();
            fixp30_t vbus_pu = ParamId_GetVbusPu();
            fixp30_t dutyQ_pu = FIXP30(0.03f);

            Duty_Ddq_t duty = {0};
            duty.D = FIXP30(0.0f);
            duty.Q = dutyQ_pu;
            MC_Set_Duty_Cycle(duty);

            if (h->subTick > h->cfg.settleTicks)
            {
                sumIq_pu += (int64_t)iq_pu;
                sumVq_pu += (int64_t)FIXP30_mpy(vbus_pu, dutyQ_pu);
            }

            if (h->subTick >= (uint16_t)(h->cfg.settleTicks + h->cfg.sampleTicks))
            {
                fixp30_t iAvg_pu = (fixp30_t)(sumIq_pu / (int64_t)h->cfg.sampleTicks);
                fixp30_t vAvg_pu = (fixp30_t)(sumVq_pu / (int64_t)h->cfg.sampleTicks);
                float iAvg = FIXP30_toF(iAvg_pu) * CURRENT_SCALE;
                float vAvg = FIXP30_toF(vAvg_pu) * VOLTAGE_SCALE;
                if (ParamId_AbsF(iAvg) > (0.1f * targetA))
                {
                    h->result.rs_ohm = vAvg / iAvg;
                    h->result.validRs = true;
                }

                h->subTick = 0U;
                if (h->requestedStep == PARAM_ID_STEP_ALL)
                {
                    h->activeStep = ParamId_NextStep(h->activeStep);
                    h->state = PARAM_ID_STATE_PREPARE;
                }
                else
                {
                    ParamId_ForceSafeOutput();
                    h->state = PARAM_ID_STATE_DONE;
                }
            }
            break;
        }

        case PARAM_ID_STEP_LD:
        case PARAM_ID_STEP_LQ:
        {
            /* Small current step, estimate L from L = (V - R*I) / (dI/dt). */
            bool isD = (h->activeStep == PARAM_ID_STEP_LD);
            float stepA = isD ? h->cfg.ldStepCurrentA : h->cfg.lqStepCurrentA;
            (void)stepA;
            fixp30_t dutyStep_pu = FIXP30(0.02f);

            Duty_Ddq_t duty = {0};
            if (isD)
            {
                duty.D = (h->subTick < h->cfg.settleTicks) ? FIXP30(0.0f) : dutyStep_pu;
                duty.Q = FIXP30(0.0f);
            }
            else
            {
                duty.D = FIXP30(0.0f);
                duty.Q = (h->subTick < h->cfg.settleTicks) ? FIXP30(0.0f) : dutyStep_pu;
            }
            MC_Set_Duty_Cycle(duty);

            if (h->subTick == h->cfg.settleTicks)
            {
                lastCurrA = isD ? ParamId_GetIdA() : ParamId_GetIqA();
                sumA = 0.0f;
                sumB = 0.0f;
            }
            else if (h->subTick > h->cfg.settleTicks)
            {
                float currA = isD ? ParamId_GetIdA() : ParamId_GetIqA();
                float dI = currA - lastCurrA;
                float dt = 1.0f / (float)TF_REGULATION_RATE;
                float didt = dI / dt;
                float v = FIXP30_toF(FIXP30_mpy(ParamId_GetVbusPu(), dutyStep_pu)) * VOLTAGE_SCALE;
                float rs = h->result.validRs ? h->result.rs_ohm : 0.0f;
                float l = 0.0f;

                if (ParamId_AbsF(didt) > 1e-3f)
                {
                    l = (v - rs * currA) / didt;
                }

                sumA += l;
                sumB += 1.0f;
                lastCurrA = currA;
            }

            if (h->subTick >= (uint16_t)(h->cfg.settleTicks + h->cfg.sampleTicks))
            {
                float lavg = (sumB > 0.0f) ? (sumA / sumB) : 0.0f;
                if (lavg > 0.0f && lavg < 1.0f)
                {
                    if (isD)
                    {
                        h->result.ld_h = lavg;
                        h->result.validLd = true;
                    }
                    else
                    {
                        h->result.lq_h = lavg;
                        h->result.validLq = true;
                    }
                }

                h->subTick = 0U;
                if (h->requestedStep == PARAM_ID_STEP_ALL && h->activeStep != PARAM_ID_STEP_KE)
                {
                    h->activeStep = ParamId_NextStep(h->activeStep);
                    h->state = PARAM_ID_STATE_PREPARE;
                }
                else
                {
                    ParamId_ForceSafeOutput();
                    h->state = PARAM_ID_STATE_DONE;
                }
            }
            break;
        }

        case PARAM_ID_STEP_KE:
        {
            /*
             * Run low-speed closed-loop speed mode and estimate Ke:
             * Ke = (Vq - Rs * Iq) / we
             */
            MC_Set_Control_Mode(CTRL_MODE_SPEED);
            MC_Set_Speed_Reference(h->cfg.keTargetSpeedRpm);

            if (h->subTick > h->cfg.settleTicks)
            {
                float vq = ParamId_GetVbusV() * FIXP30_toF(g_axis.currCtrl.outIdq.Q);
                float iq = ParamId_GetIqA();
                float rs = h->result.validRs ? h->result.rs_ohm : 0.0f;
                float we = 2.0f * 3.1415926f * FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) * FREQUENCY_SCALE;
                if (ParamId_AbsF(we) > 1e-3f)
                {
                    float ke = (vq - rs * iq) / we;
                    sumA += ke;
                    sumB += 1.0f;

                    /* Use Ke~Kt approximation in SI for J/B fit under no-load test. */
                    {
                        float kt = h->result.validKe ? h->result.ke_v_per_rad_s : ke;
                        float te = kt * iq;
                        float dt = 1.0f / (float)TF_REGULATION_RATE;
                        ParamId_JBPush(&jbCache, we, te, dt);
                    }
                }
            }

            if (h->subTick >= (uint16_t)(h->cfg.settleTicks + h->cfg.sampleTicks))
            {
                if (sumB > 0.0f)
                {
                    h->result.ke_v_per_rad_s = sumA / sumB;
                    h->result.validKe = true;
                }

                /* Finalize J/B estimation and keep result in axis parameter placeholders. */
                ParamId_JBSolve(&jbCache);
                if (jbCache.estJ > 0.0f)
                {
                    g_axis.fKt = jbCache.estJ; /* temporary runtime stash for J */
                }
                if (jbCache.estB > 0.0f)
                {
                    g_axis.fLs = jbCache.estB; /* temporary runtime stash for B */
                }

                /* Apply PI retune from identified R/L immediately after full identification. */
                ParamId_ApplyCurrentPiFromResult(&h->result);

                /* Build a complete persistence snapshot (including J/B). */
                {
                    ParamIdFlashData_t snapshot;
                    ParamId_BuildFlashData(&snapshot, &h->result, &jbCache);
                    (void)snapshot;
                }

                MC_Set_Speed_Reference(0.0f);
                ParamId_ForceSafeOutput();
                h->state = PARAM_ID_STATE_DONE;
            }
            break;
        }

        default:
            ParamId_ForceSafeOutput();
            h->state = PARAM_ID_STATE_FAULT;
            break;
    }
}

bool ParamId_SaveToFlash(const ParamIdFlashData_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    ParamIdFlashData_t blob = *data;
    blob.magic = PARAM_ID_FLASH_MAGIC;
    blob.version = PARAM_ID_FLASH_VERSION;
    blob.crc32 = 0U;
    blob.crc32 = ParamId_Crc32((const uint8_t *)&blob, (uint32_t)(sizeof(blob) - sizeof(blob.crc32)));
    return ParamId_FlashWritePage(PARAM_ID_FLASH_ADDR, &blob, (uint32_t)sizeof(blob));
}

bool ParamId_LoadFromFlash(ParamIdFlashData_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    const ParamIdFlashData_t *blob = (const ParamIdFlashData_t *)PARAM_ID_FLASH_ADDR;
    if (blob->magic != PARAM_ID_FLASH_MAGIC || blob->version != PARAM_ID_FLASH_VERSION)
    {
        return false;
    }

    uint32_t crc = ParamId_Crc32((const uint8_t *)blob, (uint32_t)(sizeof(*blob) - sizeof(blob->crc32)));
    if (crc != blob->crc32)
    {
        return false;
    }

    *data = *blob;
    return true;
}
