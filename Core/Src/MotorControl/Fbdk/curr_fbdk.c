/*
 * curr_fbdk.c
 *
 *  Created on: Apr 15, 2026
 *      Author: Administrator
 */

#include "curr_fbdk.h"
#include "stm32g4xx_ll_tim.h"
#include "motor_parameters.h"
#include "stm32g4xx_ll_system.h"
#include "main.h"

PWMC_Handle_t pwmcHandle;

CurrentFilter_t irFilter;
CurrentFilter_t isFilter;
#define ADC_SAMPLE_BLANK_TICKS   (80U)   // 死区/开关噪声避让，需实测调整
#define ADC_SAMPLE_MARGIN_TICKS  (40U)   // 离周期边界最小裕量

#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

// 预计算ADC转换常数
static const float CURR_CONV_OFFSET  = 1.65f / ((float)AMPLIFICATION_GAIN * RSHUNT);
static const float CURR_CONV_SCALE   = ADC_REFERENCE_VOLTAGE / (4096.0f * (float)AMPLIFICATION_GAIN * RSHUNT);
static const float INV_CURRENT_SCALE = 1.0f / (float)CURRENT_SCALE;
static const float VBUS_CONV_SCALE   = (3.3f * (5.1f + 100.0f)) / (4096.0f * 5.1f * (float)VOLTAGE_SCALE);

#ifdef ADC_INJ_TEST
volatile AdcInjTest_t g_adc_inj_test = {0};
void ADC_Injected_TestHook(void)
{
    SwitchOn_PWM(g_axis.pPWMCHandle);

    g_axis.pPWMCHandle->bIsMeasuringOffset = true;
    g_axis.pPWMCHandle->bIsShorting = false;

    Write_TIM_Registers(g_axis.pPWMCHandle);


    g_adc_inj_test.adc1_r1_ir   = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) >> 4;
    g_adc_inj_test.adc1_r2_is   = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) >> 4;
    g_adc_inj_test.adc1_r3_it   = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) >> 4;
    g_adc_inj_test.adc1_r4_vbus = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_4) >> 4;

    g_adc_inj_test.adc2_r1_vsenb = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1) >> 4;
    g_adc_inj_test.adc2_r2_vsenc = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) >> 4;
    g_adc_inj_test.adc2_r3_vsena = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3) >> 4;

    g_adc_inj_test.sample_cnt++;
}
#endif

/**
  * @brief  初始化TIMx、ADC以进行电流和电压读取
  * TIM1触发
  * IR-->ADC1
  * IS-->ADC4
  * VR、S、T,VBUS-->ADC2
  */
void Sampling_Init()
{
	pwmcHandle.TIMx = TIM1;
	pwmcHandle.uQuarterPeriodCnt = PWM_PERIOD_CYCLES >> 2;

	ADC_TypeDef *ADCx_1 = ADC1;
	ADC_TypeDef *ADCx_2 = ADC2;

    LL_ADC_DisableIT_EOC( ADCx_2 );
    LL_ADC_ClearFlag_EOC( ADCx_2 );
    LL_ADC_DisableIT_JEOC( ADCx_2 );
    LL_ADC_ClearFlag_JEOC( ADCx_2 );

    LL_ADC_EnableIT_EOC( ADCx_1 );
	LL_ADC_ClearFlag_EOC( ADCx_1 );
	LL_ADC_EnableIT_JEOC( ADCx_1 );
	LL_ADC_ClearFlag_JEOC( ADCx_1 );

    LL_DBGMCU_APB2_GRP1_FreezePeriph( LL_DBGMCU_APB2_GRP1_TIM1_STOP );
    LL_DBGMCU_APB1_GRP1_FreezePeriph( LL_DBGMCU_APB1_GRP1_TIM3_STOP );

    ADCxInitIT(ADCx_1);
    ADCxInitIT(ADCx_2);

    TIMxInit(TIM1);

    LL_ADC_INJ_StartConversion(ADC2);
    LL_ADC_INJ_StartConversion(ADC1);

    // 滤波系数
    irFilter.fAlpha = 0.3f;
    isFilter.fAlpha = 0.3f;


    FIXPSCALED_floatToFIXPscaled((float)(PWM_PERIOD_CYCLES >> 2), &pwmcHandle.dutySf);
}

/**
  * @brief 配置ADC采样时刻。
  * @param pHdl PWM组件当前实例的处理器
  * @rebval 错误状态错误状态
  */



static inline uint16_t min3_u16(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t m = (a < b) ? a : b;
    return (m < c) ? m : c;
}

static inline uint16_t max3_u16(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t m = (a > b) ? a : b;
    return (m > c) ? m : c;
}

/**
 * @brief  根据当前三相占空比设置下一周期ADC注入采样点（TIM1 CH4）
 * @note   适用于中心对齐PWM+三电阻采样的通用策略
 */
uint16_t Set_ADC_SampPoint_SectX(PWMC_Handle_t *pHandle)
{
    TIM_TypeDef *tim = pHandle->TIMx;
    const uint16_t half_period = (uint16_t)(PWM_PERIOD_CYCLES >> 1);

    // 当前三相比较值（已在Set_Phase_Duty中更新）
    uint16_t ta = pHandle->uCntPhR;
    uint16_t tb = pHandle->uCntPhS;
    uint16_t tc = pHandle->uCntPhT;

    uint16_t tmin = min3_u16(ta, tb, tc);
    uint16_t tmax = max3_u16(ta, tb, tc);

    // 有效采样窗口：避开开关沿 blanking
    int32_t win_start = (int32_t)tmin + (int32_t)ADC_SAMPLE_BLANK_TICKS;
    int32_t win_end   = (int32_t)tmax - (int32_t)ADC_SAMPLE_BLANK_TICKS;

    uint16_t sample_tick;

    if (win_end > win_start) {
        // 优先取窗口中点
        sample_tick = (uint16_t)((win_start + win_end) >> 1);
    } else {
        // 窗口太窄：退化到半周期中点
        sample_tick = (uint16_t)(half_period >> 1);
    }

    // 边界夹紧
    if (sample_tick < ADC_SAMPLE_MARGIN_TICKS) {
        sample_tick = ADC_SAMPLE_MARGIN_TICKS;
    }
    if (sample_tick > (half_period - ADC_SAMPLE_MARGIN_TICKS)) {
        sample_tick = (uint16_t)(half_period - ADC_SAMPLE_MARGIN_TICKS);
    }

    // 用CH4触发ADC injected（你的TIM1已配置TRGO=OC4REF）
    LL_TIM_OC_SetCompareCH4(tim, sample_tick);

    return MC_NO_FAULTS;
}


/**
  * @brief  执行空间矢量调制
  * @param  pHdl 		PWM组件当前实例的处理器
  * @param  Drst 	输入占空比
  */
uint16_t Set_Phase_Duty(PWMC_Handle_t * pHandle, Duty_Drst_t Drst)
{
	Modulate(&Drst, &pHandle->drstOut_pu);

	int32_t iTimePhR, iTimePhS, iTimePhT;

	fixpFmt_t shift = 30 + pHandle->dutySf.fixpFmt;
	iTimePhR = FIXP_MPY(pHandle->drstOut_pu.R, pHandle->dutySf.value, shift) + pHandle->uQuarterPeriodCnt;

	iTimePhS = FIXP_MPY(pHandle->drstOut_pu.S, pHandle->dutySf.value, shift) + pHandle->uQuarterPeriodCnt;

	iTimePhT = FIXP_MPY(pHandle->drstOut_pu.T, pHandle->dutySf.value, shift) + pHandle->uQuarterPeriodCnt;

	pHandle->uCntPhR = ( uint16_t )FIXP_sat(iTimePhR,(PWM_PERIOD_CYCLES >> 1),0);
	pHandle->uCntPhS = ( uint16_t )FIXP_sat(iTimePhS,(PWM_PERIOD_CYCLES >> 1),0);
	pHandle->uCntPhT = ( uint16_t )FIXP_sat(iTimePhT,(PWM_PERIOD_CYCLES >> 1),0);

	return Write_TIM_Registers(pHandle);
}

/**
  * @brief  关闭PWM输出
  * @param  pHdl PWM组件当前实例的处理器
  */
void SwitchOff_PWM(PWMC_Handle_t * pHandle)
{
	LL_TIM_CC_DisableChannel(TIM1, TIMxCCER_MASK_CH123);
	LL_TIM_DisableAllOutputs(TIM1);
}

/**
  * @brief  开启PWM输出
  * @param  pHdl PWM组件当前实例的处理器
  */
void SwitchOn_PWM(PWMC_Handle_t * pHandle)
{
	LL_TIM_EnableAllOutputs (TIM1);
	LL_TIM_CC_EnableChannel(TIM1, TIMxCCER_MASK_CH123 | LL_TIM_CHANNEL_CH3);
	LL_TIM_ClearFlag_UPDATE(TIM1);
}

/**
  * @brief  低侧接通，高侧断开，短路电机
  * @param  pHdl PWM组件当前实例的处理器
  */
void PWMShort(PWMC_Handle_t * pHandle)
{
	pHandle->bIsShorting = true;

}

/**
  * @brief  配置ADC电流读数,注入通道配置
  */
void ADCxInitIT(ADC_TypeDef * ADCx)
{
	// 退出深度关机模式
	LL_ADC_DisableDeepPowerDown(ADCx);

	if ( LL_ADC_IsInternalRegulatorEnabled(ADCx) == 0u)
	{
		// 启用ADC内部电压调节器
		LL_ADC_EnableInternalRegulator(ADCx);

		volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
		while(wait_loop_index != 0UL)
		{
			wait_loop_index--;
		}
	}

	LL_ADC_StartCalibration( ADCx, LL_ADC_SINGLE_ENDED );
	while ( LL_ADC_IsCalibrationOnGoing( ADCx) == 1u)
	{}

	// adc使能
	while (  LL_ADC_IsActiveFlag_ADRDY( ADCx ) == 0u)
	{
		LL_ADC_Enable(  ADCx );
	}

	LL_ADC_EnableIT_JEOC(ADCx);
}

/**
  * @brief  配置用于PWM生成和ADC触发的定时器
  * @param  用于PWM生成的TIMx定时器
  */
void TIMxInit( TIM_TypeDef * TIMx)
{
	uint32_t uBrk2Timeout = 1000;

	LL_TIM_DisableCounter( TIMx );

	// 在寄存器上启用TIMx预加载功能
	LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
	LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
	LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH3 );

	LL_TIM_SetCounter( TIMx, 0 );

	LL_TIM_ClearFlag_BRK( TIMx );

	while ((LL_TIM_IsActiveFlag_BRK2 (TIMx) == 1u) && (uBrk2Timeout != 0u) )
	{
	  LL_TIM_ClearFlag_BRK2( TIMx );
	  uBrk2Timeout--;
	}
	LL_TIM_EnableIT_BRK( TIMx );

	LL_TIM_ClearFlag_UPDATE(TIMx);
	LL_TIM_EnableIT_UPDATE(TIMx);
	LL_TIM_EnableCounter(TIMx);
}

/**
  * @brief  获取直流母线测量值
  * @param  pBusVoltage 直流母线测量值
  */
void Get_Vbus_Measurements(PWMC_Handle_t * pHandle, fixp30_t* pBusVoltage)
{
	uint16_t uAdcVbusValue = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_4) >> 4;
	// uint16_t uAdcVbus_VSENB = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1) >> 4;
	// uint16_t uAdcVbus_VSENC = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2) >> 4;
	// uint16_t uAdcVbus_VSENA = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3) >> 4;
	// 根据电路分析
	float fVbus = (float)uAdcVbusValue * 3.3f / 4096.0f / 5.1f * (5.1f + 100.0f);
	*pBusVoltage = FIXP30(fVbus / (float)VOLTAGE_SCALE);
}

/**
  * @brief  获取三相电流测量值
  * @param  pIrst 三相电流测量值
  */
void Get_RST_Measurements(PWMC_Handle_t * pHandle, Currents_Irst_t *pIrstMeas)
{
	uint16_t uAdcIrValue = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1) >> 4;
	float fCurrR = CURR_CONV_OFFSET - CURR_CONV_SCALE * (float)uAdcIrValue;

	uint16_t uAdcIsValue = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2) >> 4;
	float fCurrS = CURR_CONV_OFFSET - CURR_CONV_SCALE * (float)uAdcIsValue;

	uint16_t uAdcItValue = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3) >> 4;
	float fCurrt = CURR_CONV_OFFSET - CURR_CONV_SCALE * (float)uAdcItValue;
	// 低通滤波
//	CurrentFilter_Update(&irFilter, fCurrR);
//	CurrentFilter_Update(&isFilter, fCurrS);

//	pIrstMeas->R = FIXP30(irFilter.fCurr / (float)CURRENT_SCALE);
//	pIrstMeas->S = FIXP30(isFilter.fCurr / (float)CURRENT_SCALE);
	pIrstMeas->R = FIXP30(fCurrR / (float)CURRENT_SCALE);
	pIrstMeas->S = FIXP30(fCurrS / (float)CURRENT_SCALE);
	pIrstMeas->T = FIXP30(fCurrt / (float)CURRENT_SCALE);

	Curr_Offset_Handle(pHandle, pIrstMeas);

	// 清除更新标志
	LL_TIM_ClearFlag_UPDATE(pHandle->TIMx);
}

void Curr_Offset_Handle(PWMC_Handle_t * pHandle, Currents_Irst_t *pIrstMeas)
{
	if (!pHandle->bIsMeasuringOffset)
	{
		pIrstMeas->R = pIrstMeas->R - pHandle->offsetIr;
		pIrstMeas->S = pIrstMeas->S - pHandle->offsetIs;
		pIrstMeas->T = -(pIrstMeas->S + pIrstMeas->R);
		fixp30_t r = pIrstMeas->R;
		fixp30_t s = pIrstMeas->S;
		pIrstMeas->R = s;
		pIrstMeas->S = r;
	}
}

void CurrentFilter_Update(CurrentFilter_t *pCurrentFilter, float fMeasCurr)
{
	pCurrentFilter->fCurr += pCurrentFilter->fAlpha * (fMeasCurr - pCurrentFilter->fCurr);
}

uint16_t Write_TIM_Registers( PWMC_Handle_t * pHandle)
{
	uint16_t Aux;
	TIM_TypeDef * TIMx = pHandle->TIMx;

	if (pHandle->bIsShorting)
	{
		// 下桥导通（测电流）
		LL_TIM_OC_SetCompareCH1(TIMx, 0UL);
		LL_TIM_OC_SetCompareCH2(TIMx, 0UL);
		LL_TIM_OC_SetCompareCH3(TIMx, 0UL);
	}
	else if (pHandle->bIsMeasuringOffset)
	{
		LL_TIM_OC_SetCompareCH1(TIMx, TICKS_50PCT);
		LL_TIM_OC_SetCompareCH2(TIMx, TICKS_50PCT);
		LL_TIM_OC_SetCompareCH3(TIMx, TICKS_50PCT);
	}
	else
	{
		LL_TIM_OC_SetCompareCH1(TIMx, pHandle->uCntPhR);
		LL_TIM_OC_SetCompareCH2(TIMx, pHandle->uCntPhS);
		LL_TIM_OC_SetCompareCH3(TIMx, pHandle->uCntPhT);
	}

	// 判断foc的执行是否超时
	if (LL_TIM_IsActiveFlag_UPDATE(TIMx) == 1UL)
	{
		Aux = MC_DURATION;
	}
	else
	{
		Aux = MC_NO_FAULTS;
	}

	return Aux;
}
