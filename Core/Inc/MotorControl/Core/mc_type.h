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

/* ── 控制模式 ── */
typedef enum {
    CTRL_MODE_TORQUE   	= 0,   	// 力矩模式（直接设定 Iq）
    CTRL_MODE_SPEED    	= 1,   	// 速度模式
    CTRL_MODE_POSITION 	= 2,   	// 位置模式
    CTRL_MODE_OPEN_LOOP = 3,  	// 开环模式（V/F拖动、电压模式）
} ControlMode_En;

/* ── 轴状态 ──
 *
上电
 │
 ▼
UNDEFINED ──初始化完成──► IDLE
              │
              ▼
          CURR_CALIB
              │
              ▼
       ENCODER_CALIB
              │
              ▼
        AXIS_STATE_RUN ◄────────────────┐
              │                        	│
           发生故障                      	│
              ▼                        	│
        FAULT_NOW（立即关PWM）           	│
              │                        	│
           故障消除                     	│
              ▼                        	│
        FAULT_OVER ──用户确认──► IDLE ────┘
 * */
typedef enum {
    AXIS_STATE_UNDEFINED        = 0,	// 上电复位后的初始状态。此时外设还未完成初始化（HAL、SPI、DRV8353RS 等），不能进行任何控制操作。初始化完成后自动转入 IDLE
    AXIS_STATE_IDLE             = 1,	// 待机状态，PWM 输出关闭（disarm），电机无力矩。
    AXIS_STATE_OFFSET_CALIB     = 2,   	// ADC 校准
	AXIS_STATE_ENCODER_CALIB	= 3,	// 编码器校准
	AXIS_STATE_RUN      		= 4,   	// 控制循环,正常工作状态，根据 ctrl_mode 分三个子模式
    AXIS_STATE_FAULT_NOW        = 5,	// 故障，进入后立即 disarm（关 PWM 输出），防止炸管
    AXIS_STATE_FAULT_OVER       = 6,	// 故障已发生但条件消失，等待用户主动确认（发送 Fault Reset 指令，对应 CiA 402 的 Controlword bit7）后才能回到 IDLE
} AxisState_t;

/* ── 错误码（位域）── */
typedef enum {
    AXIS_ERROR_NONE                 = 0x00,
    AXIS_ERROR_OVERCURRENT          = 0x01,  // CSA SA/SB/SC_OC
    AXIS_ERROR_OVERVOLTAGE          = 0x02,  // 软件母线检测
    AXIS_ERROR_GATE_DRIVER          = 0x04,  // DRV nFAULT
    AXIS_ERROR_ENCODER              = 0x08,  // SPI 通信失败
    AXIS_ERROR_OVERTEMP             = 0x10,  // DRV OTSD 或 NTC 超温
    AXIS_ERROR_UNDERVOLTAGE         = 0x20,  // DRV UVLO
    AXIS_ERROR_CALIBRATION_FAILED   = 0x40,  // 校准超时或结果异常
} AxisError_t;


typedef struct {
    /* 设定值 */
    Currents_Idq_t refIdq;

    Currents_Irst_t IrstMeas;	// 三相电流测量值

    /* 计算重构值 */
    Currents_Iab_t calcIab;
    Currents_Idq_t calcIdq;

    /* PI 状态 */
    fixp30_t    integral_d;
    fixp30_t    integral_q;
    float    fKp;             // 比例增益
    float    fKi;             // 积分增益

    /* 输出 */
    Currents_Idq_t outIdq;

    /* 限幅 */
    fixp30_t    vMax_pu;          // 电压限幅（标幺值）

    PID_Handle_t pidId;   // d轴电流PI
    PID_Handle_t pidIq;   // q轴电流PI


    PIDREGDQX_CURRENT_s pid_IdIqX_obj;
    fixp24_t busVoltageComp;
    fixp24_t busVoltageCompMax;
    fixp24_t busVoltageCompMin;
    FIXP_scaled_t busVoltageFilter;
} CurrCtrl_t;

typedef struct {
    fixp30_t    speedRef_pu;        // 速度设定（rad/s）
    fixp30_t    speedMeas_pu;       // 来自 speed_pos_fbdk

    /* ── PI 状态 ── */
    fixp30_t	integral_pu;
    float    fKp;
    float    fKi;

    /* ── 输出（送给电流环的 Iq_ref）── */
    fixp30_t    iqOut_pu;

    /* ── 限幅 ── */
    fixp30_t    iqMax_pu;         	// 最大输出电流（限力矩）

    PID_Handle_t pid;     			// 速度PI

    PIDREG_SPEED_s	PIDSpeed;		// mcsdk得pid

    fixp30_t speedRamp_pu;			// 每次中断时应应用的速度增量
    fixp30_t speedRefRamp_pu;		// 在斜坡后的速度参考值
} SpeedCtrl_t;

typedef struct {
    float       fPosRef;        // 位置设定（角度，浮点更方便）

    float       fPosMeas;       // 来自 speed_pos_fbdk（圈数 + 角度合并）

    /* ── P 控制 ── */
    float       fKp;
    float       fKi;
    float       fKd;

    /* ── 输出（送给速度环的 speedRef_pu）── */
    fixp30_t	speedOut_pu;

    /* ── 限幅 ── */
    float       fSpeedMax;        // 输出速度限幅

    fixp30_t 	openLoopAngle_pu;	// 开环角度
    fixp30_t 	hzToStepAngle_pu;	// 每次foc执行时，累加的角度

    uint16_t    uCalibCount;		// 校准执行次数
    bool 		bCalibFlag;			// 校准状态
    uint16_t 	uOffsetAngleRaw;	// 校准偏移读取

    PID_Handle_t pid;				// 位置环PID

} PosCtrl_t;

typedef struct {
    /* 子组件 */
    SpeedAngleParam_t 	fbdk;       	// 编码器反馈（已有）
    CurrCtrl_t          currCtrl;  		// 电流控制
    SpeedCtrl_t         speedCtrl;   	// 速度控制
    PosCtrl_t           posCtrl;   		// 位置控制

    /* 状态机 */
    AxisState_t         state;
    AxisError_t         error;
    ControlMode_En      enCtrlMode;

    /* 电机参数 */
    float               fRs;         		// 相电阻（Ω）
    float               fLs;         		// 相电感（H）
    float               fKt;         		// 力矩常数（N·m/A）
    uint8_t             uPolePairs; 		// 极对数

    /* 母线电压 */
    fixp30_t            busVoltage;         // 母线电压（V）

    bool 				bMCBootCompleted;	// 电机初始化完成标志

    SafetyConfig_t 		safetyConfig;		// 安全阈值配置

    PWMC_Handle_t *		pPWMCHandle;

} Axis_t;


/**
  * @brief  低侧或使能信号定义
  */
typedef enum
{
  LS_DISABLED  = 0x0U,   /* 低侧信号和使能信号始终关闭。这等同于DISABLED */
  LS_PWM_TIMER = 0x1U,   /* 低边PWM信号由定时器生成。等同于ENABLED */
  ES_GPIO   = 0x2U       /* 启用信号由GPIO管理(L6230模式) */
} LowSideOutputsFunction_t;

/**
 * 故障代码
 * 以下符号定义了与电机控制子系统可能引发的故障相关的代码。
 * 电机控制子系统可以触发。
 */
#define  MC_NO_ERROR     ((uint16_t)0x0000) /**< @brief No error. */
#define  MC_NO_FAULTS    ((uint16_t)0x0000) /**< @brief No error. */
#define  MC_DURATION     ((uint16_t)0x0001) /**< @brief Error: FOC rate to high. */
#define  MC_OVER_VOLT    ((uint16_t)0x0002) /**< @brief Error: Software over voltage. */
#define  MC_UNDER_VOLT   ((uint16_t)0x0004) /**< @brief Error: Software under voltage. */
#define  MC_OVER_TEMP    ((uint16_t)0x0008) /**< @brief Error: Software over temperature. */
#define  MC_START_UP     ((uint16_t)0x0010) /**< @brief Error: Startup failed. */
#define  MC_SPEED_FDBK   ((uint16_t)0x0020) /**< @brief Error: Speed feedback. */
#define  MC_OVER_CURR    ((uint16_t)0x0040) /**< @brief Error: Emergency input (Over current). */
#define  MC_SW_ERROR     ((uint16_t)0x0080) /**< @brief Software Error. */
#define  MC_SAMPLEFAULT (uint16_t)(0x0100u)    /**< @brief Error: Fewer than two phase current samples available */
#define  MC_OVERCURR_SW (uint16_t)(0x0200u)    /**< @brief Error: Software overcurrent tripped */
#define  MC_DP_FAULT     ((uint16_t)0x0400) /**< @brief Error Driver protection fault. */

#endif /* INC_MOTORCONTROL_MC_TYPE_H_ */
