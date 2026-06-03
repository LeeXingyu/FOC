/*
 * curr_fbdk.h
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_FBDK_CURR_FBDK_H_
#define INC_MOTORCONTROL_FBDK_CURR_FBDK_H_

#include "fixpmath.h"
#include "stm32g4xx_hal.h"

/**
  * @brief 此结构用于处理PWM与电流反馈组件实例的数据
  *
  */
typedef struct
{
  TIM_TypeDef * TIMx;

  uint16_t                                uCntPhR;                           /*!< @brief PWM Duty cycle for phase A */
  uint16_t                                uCntPhS;                           /*!< @brief PWM Duty cycle for phase B */
  uint16_t                                uCntPhT;                           /*!< @brief PWM Duty cycle for phase C */
  uint16_t                                uSWerror;                          /*!< @brief Contains status about SW error */
  bool                                    bTurnOnLowSidesAction;             /*!< @brief true if TurnOnLowSides action is active, false otherwise. */
  uint16_t                                uOffCalibrWaitTimeCounter;         /*!< @brief Counter to wait fixed time before motor
                                                                                 current measurement offset calibration. */
  uint8_t                                 uMotor;                            /*!< @brief Motor reference number */
  uint16_t                                uQuarterPeriodCnt;                 /*!< @brief Quarter period. This is the compare value for 50% duty */
  FIXP_scaled_t                           dutySf;                          /*!< @brief Scale factor of QuarterPeriodCnt */

  bool                                    bOverCurrentFlag;                  /* This flag is set when an overcurrent occurs.*/
  bool                                    bOverVoltageFlag;                  /* This flag is set when an overvoltage occurs.*/
  bool                                    bDriverProtectionFlag;             /* This flag is set when a driver protection occurs.*/
  bool                                    bBrakeActionLock;                  /* This flag is set to avoid that brake action is interrupted.*/

  bool 									  bIsShorting;						 /* 是否下桥短路 */
  bool 									  bIsMeasuringOffset;				 // 电流校准标志

  float 									fCalibRsum;
  float 									fCalibSsum;
  uint16_t 									uCalibCount;


  fixp30_t			offsetIr;			// R相电流校准偏移值
  fixp30_t			offsetIs;			// S相电流校准偏移值


  Duty_Drst_t                             drstUnmodulatedOut_pu;          /*!< @brief three phase duty cycles before modulation */
  Duty_Drst_t                             drstOut_pu;                      /*!< @brief three phase duty cycles after modulation */



} PWMC_Handle_t;

extern PWMC_Handle_t pwmcHandle;


/**
 * @brief 测量数据结构定义
 */
typedef struct
{
  Currents_Iab_t      Iab_pu;      /* 测量的电流Ia、Iß */
  Voltages_Uab_t      Uab_pu;      /* 测量的电压Va、Vß */
}Measurement_Output_t;

typedef struct {
    float fCurr;
    float fAlpha;
} CurrentFilter_t;

typedef struct
{
  Currents_Irst_t	Irst_in_pu;
  Currents_Iab_t   	Iab_in_pu;                      /*!< @brief input current in ab coordinates expressed in per-unit */
  Voltages_Uab_t   	Uab_emf_pu;                     /*!< @brief input Bemf in ab coordinates expressed in per-unit */
  Currents_Idq_t   	Idq_ref_currentcontrol_pu;      /*!< @brief current reference in dq coordinates expressed in per-unit */
  fixp30_t 			SpeedLP;                                /*!< @brief motor electrical frequency */
  fixp30_t 			Udcbus_in_pu;                           /*!< @brief DC bus level expressed in per-unit */
  bool 				pwmControlEnable;                           /*!< @brief PWM control flag status */
} CurrCtrlInput_t;
#define ADC_INJ_TEST
#ifdef ADC_INJ_TEST
typedef struct
{
    uint16_t adc1_r1_ir;
    uint16_t adc1_r2_is;
    uint16_t adc1_r3_it;
    uint16_t adc1_r4_vbus;

    uint16_t adc2_r1_vsenb;
    uint16_t adc2_r2_vsenc;
    uint16_t adc2_r3_vsena;

    Currents_Irst_t irst_meas;
    fixp30_t bus_voltage_pu;

    uint32_t sample_cnt;
    uint32_t update_cnt;
    uint32_t stale_cnt;

    uint16_t last_adc1_r1_ir;
    uint16_t last_adc1_r2_is;
    uint16_t last_adc1_r3_it;
    uint16_t last_adc1_r4_vbus;
    uint16_t last_adc2_r1_vsenb;
    uint16_t last_adc2_r2_vsenc;
    uint16_t last_adc2_r3_vsena;
} AdcInjTest_t;
void ADC_Injected_TestHook(void);
#endif
/**
  * @brief  初始化TIMx、ADC以进行电流和电压读取
  */
void Sampling_Init();

/**
  * @brief  配置ADC电流读数
  */
void ADCxInitIT(ADC_TypeDef * ADCx);

/**
  * @brief  执行空间矢量调制
  * @param  pHdl 		PWM组件当前实例的处理器
  * @param  Drst	 	输入占空比
  */
uint16_t Set_Phase_Duty(PWMC_Handle_t * pHandle, Duty_Drst_t Drst);

/**
  * @brief 配置ADC采样时刻。
  * @param pHdl PWM组件当前实例的处理器
  * @rebval 错误状态错误状态
  */
uint16_t Set_ADC_SampPoint_SectX(PWMC_Handle_t * pHandle);

/**
  * @brief  关闭PWM输出
  * @param  pHdl PWM组件当前实例的处理器
  */
void SwitchOff_PWM(PWMC_Handle_t * pHandle);

/**
  * @brief  开启PWM输出
  * @param  pHdl PWM组件当前实例的处理器
  */
void SwitchOn_PWM(PWMC_Handle_t * pHandle);

/**
  * @brief  获取直流母线测量值
  * @param  pBusVoltage 直流母线测量值
  */
void Get_Vbus_Measurements(PWMC_Handle_t * pHandle, fixp30_t* pBusVoltage);

/**
  * @brief  获取三相电流测量值
  * @param  pCurrCtrl 电流控制句柄
  */
void Get_RST_Measurements(PWMC_Handle_t * pHandle, Currents_Irst_t *pIrstMeas);

/**
  * @brief  一阶低通滤波过滤电流
  * @param  pFilterCurr 过滤电流
  * @param  fMeasCurr 	采样电流
  */
void CurrentFilter_Update(CurrentFilter_t *pCurrentFilter, float fMeasCurr);

/**
  * @brief  将新的占空比写入外设寄存器
  */
uint16_t Write_TIM_Registers( PWMC_Handle_t * pHandle);

#endif /* INC_MOTORCONTROL_FBDK_CURR_FBDK_H_ */
