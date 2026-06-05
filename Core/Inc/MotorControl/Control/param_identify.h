#ifndef INC_MOTORCONTROL_CONTROL_PARAM_IDENTIFY_H_
#define INC_MOTORCONTROL_CONTROL_PARAM_IDENTIFY_H_

#include <stdbool.h>
#include <stdint.h>
#include "mc_type.h"

/**
 * @brief Parameter identification sub-task.
 *
 * This module estimates:
 * - Phase resistance Rs
 * - D-axis inductance Ld
 * - Q-axis inductance Lq
 * - Back-EMF constant Ke (line model in q-axis)
 *
 * Design goal:
 * - Keep low-level drivers untouched.
 * - Reuse existing control/feedback interfaces in current project.
 */

typedef enum
{
    PARAM_ID_STEP_RS = 0,
    PARAM_ID_STEP_LD,
    PARAM_ID_STEP_LQ,
    PARAM_ID_STEP_KE,
    PARAM_ID_STEP_ALL
} ParamIdStep_t;

typedef enum
{
    PARAM_ID_STATE_IDLE = 0,
    PARAM_ID_STATE_PREPARE,
    PARAM_ID_STATE_LOCK_CHECK,
    PARAM_ID_STATE_RUN,
    PARAM_ID_STATE_DONE,
    PARAM_ID_STATE_FAULT
} ParamIdState_t;

typedef enum
{
    PARAM_ID_OK = 0,
    PARAM_ID_ERR_BUSY,
    PARAM_ID_ERR_NOT_RUNNING,
    PARAM_ID_ERR_BAD_ARG
} ParamIdRet_t;

typedef struct
{
    bool validRs;
    bool validLd;
    bool validLq;
    bool validKe;

    float rs_ohm;
    float ld_h;
    float lq_h;
    float ke_v_per_rad_s;
} ParamIdResult_t;

typedef struct
{
    float i_avg_a;
    float v_avg_v;
    float calc_id_a;
    float calc_iq_a;
} ParamIdDebugData_t;

typedef struct
{
    /* Common limits */
    float maxCurrentA;
    float maxSpeedRpm;
    uint32_t maxRunTicks;

    /* Excitation settings */
    float rsCurrentA;
    float ldStepCurrentA;
    float lqStepCurrentA;
    float keTargetSpeedRpm;

    /* Sample windows */
    uint16_t settleTicks;
    uint16_t sampleTicks;

    /* Rotor lock check window for Rs/Ld/Lq */
    uint16_t lockCheckTicks;
    uint16_t lockMaxAngleDeltaRaw;
    float lockMaxSpeedRpm;
} ParamIdConfig_t;

typedef struct
{
    ParamIdState_t state;
    ParamIdStep_t requestedStep;
    ParamIdStep_t activeStep;
    uint32_t tick;
    uint16_t subTick;
    bool stopRequested;
    ParamIdResult_t result;
    ParamIdConfig_t cfg;
} ParamIdHandle_t;

/**
 * @brief Flash storage layout for identified parameters.
 * magic/version are used to validate payload.
 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    /*
     * Unified persisted payload:
     * 1) identified electrical parameters
     * 2) computed current-loop PI tuning parameters
     * 3) identified mechanical parameters
     */
    ParamIdResult_t result;
    uint32_t pole_pairs;
    uint8_t can_node_id;
    uint8_t reserved0[3];
    float curr_kp_si;
    float curr_wi_si;
    /* Mechanical parameters identified from speed/torque data */
    float mech_j;   /* inertia J, unit: kg*m^2 */
    float mech_b;   /* viscous coefficient B, unit: N*m*s/rad */
    uint32_t crc32;
} ParamIdFlashData_t;

void ParamId_Init(ParamIdHandle_t *h);
ParamIdRet_t ParamId_Start(ParamIdHandle_t *h, ParamIdStep_t step);
ParamIdRet_t ParamId_Stop(ParamIdHandle_t *h);
void ParamId_Service(ParamIdHandle_t *h);
ParamIdState_t ParamId_GetState(const ParamIdHandle_t *h);
const ParamIdResult_t *ParamId_GetResult(const ParamIdHandle_t *h);
void ParamId_GetDebugData(ParamIdDebugData_t *data);
void ParamId_GetDefaultConfig(ParamIdConfig_t *cfg);
ParamIdRet_t ParamId_SetConfig(ParamIdHandle_t *h, const ParamIdConfig_t *cfg);

/* Flash storage API */
bool ParamId_SaveToFlash(const ParamIdFlashData_t *data);
bool ParamId_LoadFromFlash(ParamIdFlashData_t *data);
bool ParamId_ClearFlash(void);
bool ParamId_ApplyFlashDataToAxis(const ParamIdFlashData_t *data);
bool ParamId_RestoreFromFlashToAxis(void);
uint8_t ParamId_GetCanNodeId(void);
bool ParamId_SetCanNodeId(uint8_t nodeId);
bool ParamId_SaveCanNodeIdToFlash(uint8_t nodeId);

void ParamId_ModuleInit(void);
ParamIdRet_t ParamId_ModuleStart(ParamIdStep_t step);
ParamIdRet_t ParamId_ModuleStop(void);
void ParamId_ModuleService(void);
void ParamId_ModuleBackgroundService(void);
ParamIdState_t ParamId_ModuleGetState(void);
const ParamIdResult_t *ParamId_ModuleGetResult(void);


#endif /* INC_MOTORCONTROL_CONTROL_PARAM_IDENTIFY_H_ */
