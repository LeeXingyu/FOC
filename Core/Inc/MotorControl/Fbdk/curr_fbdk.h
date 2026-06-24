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
  * @brief 姝ょ粨鏋勭敤浜庡鐞哖WM涓庣數娴佸弽棣堢粍浠跺疄渚嬬殑鏁版嵁
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

  bool 									  bIsShorting;						 /* 鏄惁涓嬫ˉ鐭矾 */
  bool 									  bIsMeasuringOffset;				 // 鐢垫祦鏍″噯鏍囧織

  float fCalibRsum;
  float fCalibSsum;
  float fCalibTsum;
  uint16_t uCalibCount;


  fixp30_t offsetIr;
  fixp30_t offsetIs;
  fixp30_t offsetIt;
  Duty_Drst_t                             drstUnmodulatedOut_pu;          /*!< @brief three phase duty cycles before modulation */
  Duty_Drst_t                             drstOut_pu;                      /*!< @brief three phase duty cycles after modulation */



} PWMC_Handle_t;

extern PWMC_Handle_t pwmcHandle;


/**
 * @brief 娴嬮噺鏁版嵁缁撴瀯瀹氫箟
 */
typedef struct
{
  Currents_Iab_t      Iab_pu;      /* 娴嬮噺鐨勭數娴両a銆両脽 */
  Voltages_Uab_t      Uab_pu;      /* 娴嬮噺鐨勭數鍘媀a銆乂脽 */
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
//#define ADC_INJ_TEST
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
  * @brief  鍒濆鍖朤IMx銆丄DC浠ヨ繘琛岀數娴佸拰鐢靛帇璇诲彇
  */
void Sampling_Init();

/**
  * @brief  閰嶇疆ADC鐢垫祦璇绘暟
  */
void ADCxInitIT(ADC_TypeDef * ADCx);

/**
  * @brief  鎵ц绌洪棿鐭㈤噺璋冨埗
  * @param  pHdl 		PWM缁勪欢褰撳墠瀹炰緥鐨勫鐞嗗櫒
  * @param  Drst	 	杈撳叆鍗犵┖姣?  */
uint16_t Set_Phase_Duty(PWMC_Handle_t * pHandle, Duty_Drst_t Drst);

/**
  * @brief 閰嶇疆ADC閲囨牱鏃跺埢銆?  * @param pHdl PWM缁勪欢褰撳墠瀹炰緥鐨勫鐞嗗櫒
  * @rebval 閿欒鐘舵€侀敊璇姸鎬?  */
uint16_t Set_ADC_SampPoint_SectX(PWMC_Handle_t * pHandle);

/**
  * @brief  鍏抽棴PWM杈撳嚭
  * @param  pHdl PWM缁勪欢褰撳墠瀹炰緥鐨勫鐞嗗櫒
  */
void SwitchOff_PWM(PWMC_Handle_t * pHandle);

/**
  * @brief  寮€鍚疨WM杈撳嚭
  * @param  pHdl PWM缁勪欢褰撳墠瀹炰緥鐨勫鐞嗗櫒
  */
void SwitchOn_PWM(PWMC_Handle_t * pHandle);

/**
  * @brief  鑾峰彇鐩存祦姣嶇嚎娴嬮噺鍊?  * @param  pBusVoltage 鐩存祦姣嶇嚎娴嬮噺鍊?  */
void Get_Vbus_Measurements(PWMC_Handle_t * pHandle, fixp30_t* pBusVoltage);

/**
  * @brief  鑾峰彇涓夌浉鐢垫祦娴嬮噺鍊?  * @param  pCurrCtrl 鐢垫祦鎺у埗鍙ユ焺
  */
void Get_RST_Measurements(PWMC_Handle_t * pHandle, Currents_Irst_t *pIrstMeas);

/**
  * @brief  涓€闃朵綆閫氭护娉㈣繃婊ょ數娴?  * @param  pFilterCurr 杩囨护鐢垫祦
  * @param  fMeasCurr 	閲囨牱鐢垫祦
  */
void CurrentFilter_Update(CurrentFilter_t *pCurrentFilter, float fMeasCurr);

/**
  * @brief  灏嗘柊鐨勫崰绌烘瘮鍐欏叆澶栬瀵勫瓨鍣?  */
uint16_t Write_TIM_Registers( PWMC_Handle_t * pHandle);

#endif /* INC_MOTORCONTROL_FBDK_CURR_FBDK_H_ */

