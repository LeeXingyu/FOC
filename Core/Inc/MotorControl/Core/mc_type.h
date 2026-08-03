/*
 * mc_type.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_MC_TYPE_H_
#define INC_MOTORCONTROL_MC_TYPE_H_

#include "Utils/Fixp/fixpmath.h"
#include "MotorControl/Fbdk/speed_pos_type.h"
#include "MotorControl/Safety/safety_type.h"
#include "curr_fbdk.h"
#include "pid.h"
#include "pidregdqx_current.h"
#include "pidreg_speed.h"
#include "pid_position_type.h"

/* 控制模式 */
typedef enum {
    CTRL_MODE_TORQUE   = 0,
    CTRL_MODE_SPEED    = 1,
    CTRL_MODE_POSITION = 2,
    CTRL_MODE_OPEN_LOOP = 3,
} ControlMode_En;

/* 轴状态 */
typedef enum {
    AXIS_STATE_UNDEFINED        = 0,
    AXIS_STATE_IDLE             = 1,
    AXIS_STATE_OFFSET_CALIB     = 2,
    AXIS_STATE_ENCODER_CALIB    = 3,
    AXIS_STATE_PARAM_CALIB      = 4,
    AXIS_STATE_RUN              = 5,
    AXIS_STATE_FAULT_NOW        = 6,
    AXIS_STATE_FAULT_OVER       = 7,
    AXIS_STATE_CURRENT_AUTOTUNE = 8,
    AXIS_STATE_SPEED_AUTOTUNE   = 9,
} AxisState_t;

/* 轴错误 */
typedef enum {
    AXIS_ERROR_NONE               = 0x00,
    AXIS_ERROR_OVERCURRENT        = 0x01,
    AXIS_ERROR_OVERVOLTAGE        = 0x02,
    AXIS_ERROR_GATE_DRIVER        = 0x04,
    AXIS_ERROR_ENCODER            = 0x08,
    AXIS_ERROR_OVERTEMP           = 0x10,
    AXIS_ERROR_UNDERVOLTAGE       = 0x20,
    AXIS_ERROR_CALIBRATION_FAILED = 0x40,
} AxisError_t;

typedef struct {
    Currents_Idq_t refIdq;
    Currents_Irst_t IrstMeas;
    Currents_Iab_t calcIab;
    Currents_Idq_t calcIdq;
    Currents_Idq_t outIdq;
    fixp30_t vMax_pu;
    PID_Handle_t pidId;
    PID_Handle_t pidIq;
    union
    {
        PIDREGDQX_CURRENT_s pid_IdIqX_obj;
        PIDREGDQX_CURRENT_s pidIdIq;
    };
    fixp24_t busVoltageComp;
    fixp24_t busVoltageCompMax;
    fixp24_t busVoltageCompMin;
    FIXP_scaled_t busVoltageFilter;
} CurrCtrl_t;

typedef struct {
    Voltages_Urst_t VrstMeas;
    Voltages_Uab_t calcVab;
    Voltages_Udq_t calcVdq;
} VotlMeas_t;

typedef struct {
    fixp30_t speedRef_pu;
    fixp30_t speedMeas_pu;
    fixp30_t integral_pu;
    float fKp;
    float fKi;
    fixp30_t iqOut_pu;
    fixp30_t iqMax_pu;
    PID_Handle_t pid;
    PIDREG_SPEED_s PIDSpeed;
    fixp30_t speedRamp_pu;
    fixp30_t speedRefRamp_pu;
} SpeedCtrl_t;

typedef struct {
    float fPosRef;
    float fPosMeas;
    float fKp;
    float fKi;
    float fKd;
    fixp30_t speedOut_pu;
    float fSpeedMax;
    fixp30_t openLoopAngle_pu;
    fixp30_t hzToStepAngle_pu;
    uint16_t uCalibCount;
    bool bCalibFlag;
    union
    {
        uint32_t uOffsetAngleRawNative;
        uint32_t uOffsetAngleRaw;
    };
    bool bResetFlag;
    int64_t iAbsRawPos;
    int32_t iZeroAngle;
    uint16_t uCircle;
    Position_Handle_t positionHandle;
    TrajPlanner_t traj;
} PosCtrl_t;

typedef struct {
    SpeedAngleParam_t fbdk;
    CurrCtrl_t currCtrl;
    SpeedCtrl_t speedCtrl;
    PosCtrl_t posCtrl;
    VotlMeas_t VotlMeas;
    AxisState_t state;
    AxisError_t error;
    ControlMode_En enCtrlMode;
    float fRs;
    float fLs;
    float fLq;
    float fKt;
    uint8_t uPolePairs;
    fixp30_t busVoltage;
    bool bMCBootCompleted;
    SafetyConfig_t safetyConfig;
    PWMC_Handle_t *pPWMCHandle;
    uint16_t uFaultNow;
    uint16_t uFaultOccurred;
} Axis_t;

#define  MC_NO_ERROR     ((uint16_t)0x0000)
#define  MC_NO_FAULTS    ((uint16_t)0x0000)
#define  MC_DURATION     ((uint16_t)0x0001)
#define  MC_OVER_VOLT    ((uint16_t)0x0002)
#define  MC_UNDER_VOLT   ((uint16_t)0x0004)
#define  MC_OVER_TEMP    ((uint16_t)0x0008)
#define  MC_START_UP     ((uint16_t)0x0010)
#define  MC_SPEED_FDBK   ((uint16_t)0x0020)
#define  MC_OVER_CURR    ((uint16_t)0x0040)
#define  MC_SW_ERROR     ((uint16_t)0x0080)
#define  MC_SAMPLEFAULT  ((uint16_t)0x0100)
#define  MC_OVERCURR_SW  ((uint16_t)0x0200)
#define  MC_DP_FAULT     ((uint16_t)0x0400)

typedef enum
{
  LS_DISABLED  = 0x0U,
  LS_PWM_TIMER = 0x1U,
  ES_GPIO      = 0x2U
} LowSideOutputsFunction_t;

#endif /* INC_MOTORCONTROL_MC_TYPE_H_ */
