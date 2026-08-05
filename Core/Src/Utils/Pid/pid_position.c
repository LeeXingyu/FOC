/*
 * pid_position.c
 */

#include "pid_position.h"
#include "motor_parameters.h"
#include "speed_autotune.h"
#include <math.h>

static void Position_TrajUpdate_T(TrajPlanner_t *pTrajPlanner,
        int64_t iMeasuredAngle,
        int64_t iTargetPosition)
{
    if (pTrajPlanner == NULL)
    {
        return;
    }

    if (!pTrajPlanner->inited)
    {
        pTrajPlanner->pos_ref = (float)iMeasuredAngle;
        pTrajPlanner->vel_ref = 0.0f;
        pTrajPlanner->inited = true;
    }

    float fError = (float)iTargetPosition - pTrajPlanner->pos_ref;
    float fError16 = fError / (float)ENC_COUNTS_PER_REV;

    if (fabsf(fError16) < POSITION_TRAJ_DONE_COUNT)
    {
        pTrajPlanner->pos_ref = (float)iTargetPosition;
        pTrajPlanner->vel_ref = 0.0f;
        return;
    }

    float fDecelDist = (pTrajPlanner->vel_ref * pTrajPlanner->vel_ref)
            / (2.0f * pTrajPlanner->acc_max);

    float fDesiredVel;
    if (fabsf(fError) <= fDecelDist)
    {
        float fValue = sqrtf(2.0f * pTrajPlanner->acc_max * fabsf(fError));
        fDesiredVel = (fError > 0.0f) ? fValue : -fValue;
        pTrajPlanner->dbg_phase = 'D';
    }
    else
    {
        fDesiredVel = (fError > 0.0f) ? pTrajPlanner->vel_max : -pTrajPlanner->vel_max;
        pTrajPlanner->dbg_phase = (fabsf(pTrajPlanner->vel_ref) >= pTrajPlanner->vel_max * 0.99f) ? 'C' : 'A';
    }

    float fDv = fDesiredVel - pTrajPlanner->vel_ref;
    if (fDv > pTrajPlanner->acc_max)
    {
        fDv = pTrajPlanner->acc_max;
    }
    else if (fDv < -pTrajPlanner->acc_max)
    {
        fDv = -pTrajPlanner->acc_max;
    }

    pTrajPlanner->vel_ref += fDv;
    pTrajPlanner->pos_ref += pTrajPlanner->vel_ref;

    if ((fError > 0.0f && pTrajPlanner->pos_ref > (float)iTargetPosition) ||
        (fError < 0.0f && pTrajPlanner->pos_ref < (float)iTargetPosition))
    {
        pTrajPlanner->pos_ref = (float)iTargetPosition;
        pTrajPlanner->vel_ref = 0.0f;
    }
}

static float SmoothStep(float x, float edge0, float edge1)
{
    float t = (x - edge0) / (edge1 - edge0);
    if (t <= 0.0f)
    {
        return 0.0f;
    }
    if (t >= 1.0f)
    {
        return 1.0f;
    }
    return t * t * (3.0f - 2.0f * t);
}

float Pid_Position_Run(PosCtrl_t *pPosCtrl)
{
    int64_t iTargetPosition = (int64_t)pPosCtrl->fPosRef;
    int64_t iMeasuredAngle = pPosCtrl->iAbsRawPos - pPosCtrl->iZeroAngle;
    int64_t iError = iTargetPosition - iMeasuredAngle;
    float fTargetError16 = (float)iError / (float)ENC_COUNTS_PER_REV;
    float fAbsTargetError16 = fabsf(fTargetError16);
    float fPositionPeriod = 1.0f / (float)SPEED_CONTROL_RATE;

    int64_t iRefForPid;
    float fFFRpm;
    if (pPosCtrl->traj.bEnable)
    {
        Position_TrajUpdate_T(&pPosCtrl->traj, iMeasuredAngle, iTargetPosition);
        iRefForPid = (int64_t)pPosCtrl->traj.pos_ref;
        fFFRpm = COUNTS_MS_TO_RPM(pPosCtrl->traj.vel_ref);
    }
    else
    {
        pPosCtrl->traj.inited = false;
        pPosCtrl->traj.pos_ref = (float)iMeasuredAngle;
        pPosCtrl->traj.vel_ref = 0.0f;
        iRefForPid = iTargetPosition;
        fFFRpm = 0.0f;
    }

    static int64_t s_prevErrorSign = 0;
    static float s_oscEnergy = 0.0f;
    static uint16_t s_calmCounter = 0U;

    s_oscEnergy *= 0.998f;
    if (fAbsTargetError16 > POSITION_OSC_CROSS_COUNT)
    {
        int64_t iCurSign = (iError > 0) ? 1 : -1;
        if ((s_prevErrorSign != 0) && (iCurSign != s_prevErrorSign))
        {
            s_oscEnergy += 2.0f;
        }
        s_prevErrorSign = iCurSign;
    }

    if (fAbsTargetError16 < POSITION_OSC_CALM_COUNT)
    {
        if (++s_calmCounter >= 100U)
        {
            s_oscEnergy = 0.0f;
            s_calmCounter = 0U;
        }
    }
    else
    {
        s_calmCounter = 0U;
    }

    bool bOscillating = (s_oscEnergy > 2.0f);
    int64_t iErrorAngle = iRefForPid - iMeasuredAngle;

    float score = g_speedAutoTuneResult.lowMobilityScore;
    float weight = score * score * (3.0f - 2.0f * score);
    float positionGainFar = POSITION_GAIN_FAR_MIN
            + (POSITION_GAIN_FAR_MAX - POSITION_GAIN_FAR_MIN) * weight;
    float positionGainNear = POSITION_GAIN_NEAR_MIN
            + (POSITION_GAIN_NEAR_MAX - POSITION_GAIN_NEAR_MIN) * weight;

    float fGainBlend = 1.0f - SmoothStep(fAbsTargetError16,
            POSITION_GAIN_BLEND_INNER_COUNT,
            POSITION_GAIN_BLEND_OUTER_COUNT);
    float fPositionGain = positionGainFar
            + (positionGainNear - positionGainFar) * fGainBlend;

    pPosCtrl->positionHandle.Position_Gain = (uint32_t)(fPositionGain + 0.5f);

    float fCorrection = (float)iErrorAngle
            * fPositionGain
            / (float)pPosCtrl->positionHandle.Postiion_Div;
    if (fCorrection > 100.0f)
    {
        fCorrection = 100.0f;
    }
    else if (fCorrection < -100.0f)
    {
        fCorrection = -100.0f;
    }

    float fIntegralDecay = expf(-fPositionPeriod / POSITION_CAPTURE_I_DECAY_TIME_S);
    bool bCrossedTarget = ((pPosCtrl->positionHandle.prev_target_error > 0)
            && (iError < 0))
            || ((pPosCtrl->positionHandle.prev_target_error < 0)
            && (iError > 0));
    bool bCaptureIntegral = (fAbsTargetError16 < POSITION_CAPTURE_ERROR_COUNT)
            && (fabsf(fFFRpm) < POSITION_CAPTURE_FF_MAX_RPM)
            && !bOscillating;

    if (bCrossedTarget)
    {
        pPosCtrl->positionHandle.pos_integral *= POSITION_CAPTURE_ZERO_CROSS_DECAY;
    }

    if (bCaptureIntegral
            && (fAbsTargetError16 > POSITION_CAPTURE_HOLD_COUNT)
            && (pPosCtrl->positionHandle.Position_Ki_Div != 0U))
    {
        float fKi = (float)pPosCtrl->positionHandle.Position_Ki
                / (float)pPosCtrl->positionHandle.Position_Ki_Div;
        pPosCtrl->positionHandle.pos_integral +=
                fTargetError16 * fKi * fPositionPeriod;
    }
    else
    {
        pPosCtrl->positionHandle.pos_integral *= fIntegralDecay;
    }

    float fIntegralLimit = pPosCtrl->positionHandle.pos_integral_lim;
    if (pPosCtrl->positionHandle.pos_integral > fIntegralLimit)
    {
        pPosCtrl->positionHandle.pos_integral = fIntegralLimit;
    }
    else if (pPosCtrl->positionHandle.pos_integral < -fIntegralLimit)
    {
        pPosCtrl->positionHandle.pos_integral = -fIntegralLimit;
    }
    pPosCtrl->positionHandle.prev_target_error = iError;
    pPosCtrl->positionHandle.prev_error_angle = (int32_t)iErrorAngle;

    float fDerivOutput = 0.0f;
    (void)fDerivOutput;

    float fCorrectionWithIntegral = fCorrection
            + pPosCtrl->positionHandle.pos_integral + fDerivOutput;
    float fCorrectionStepMax = POSITION_CORRECTION_SLEW_RPM_PER_S * fPositionPeriod;
    float fCorrectionStep = fCorrectionWithIntegral
            - pPosCtrl->positionHandle.pos_correction_prev;
    if (fCorrectionStep > fCorrectionStepMax)
    {
        fCorrectionStep = fCorrectionStepMax;
    }
    else if (fCorrectionStep < -fCorrectionStepMax)
    {
        fCorrectionStep = -fCorrectionStepMax;
    }
    pPosCtrl->positionHandle.pos_correction_prev += fCorrectionStep;

    float hSpeedReference = fFFRpm + pPosCtrl->positionHandle.pos_correction_prev;
    float fTrajLim = COUNTS_MS_TO_RPM(pPosCtrl->traj.vel_max);
    if (hSpeedReference > fTrajLim)
    {
        hSpeedReference = fTrajLim;
    }
    else if (hSpeedReference < -fTrajLim)
    {
        hSpeedReference = -fTrajLim;
    }

    return hSpeedReference;
}
