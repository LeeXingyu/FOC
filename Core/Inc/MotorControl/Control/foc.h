/*
 * foc.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_FOC_H_
#define INC_MOTORCONTROL_FOC_H_

#include "fixpmath_types.h"
#include "fixpmath.h"

/*
 * @brief Park变换
 * a、b -> d、q
 * */
void Park_Current(const Currents_Iab_t Iab, FIXP_CosSin_t* cosSin, Currents_Idq_t * pIdq);

/*
 * @brief 反Park变换
 * */
void Inv_Park_Duty( Duty_Ddq_t Ddq, FIXP_CosSin_t* cosSin, Duty_Dab_t * pDab);

/*
 * @brief Clarke变换
 * r、s、t -> a、b
 * */
void Clarke_Current( const Currents_Irst_t Irst, Currents_Iab_t *pIab);

/*
 * @brief 反Clarke变换
 * */
void Inv_Clarke_Duty(Duty_Dab_t dutyDab, Duty_Drst_t *pDutyDrst);

/*
 * @brief 空间矢量调制
 * 空间矢量调制通过减去最高和最低占空比的平均值，将信号集中在可用电压范围的中间位;等效SVPWM，但是计算效率更高
 * */
void Modulate(const Duty_Drst_t *pDutyDrstInput, Duty_Drst_t *pDutyDrstOutput);

#endif /* INC_MOTORCONTROL_FOC_H_ */
