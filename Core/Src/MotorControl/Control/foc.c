/*
 * foc.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#include "foc.h"

#define SQRT_3_OVER_2				(0.86602540378f)

/*
 * Park变换
 * a、b -> d、q
 * */
void Park_Current(const Currents_Iab_t Iab, FIXP_CosSin_t* cosSin, Currents_Idq_t * pIdq)
{
//	Idq.D = Iab.A * cos(theta) + Iab.B * sin(theta);
	pIdq->D = FIXP30_mpy(Iab.A, cosSin->cos) + FIXP30_mpy(Iab.B, cosSin->sin);

//	Idq.Q = Iab.B * cos(theta) - Iab.A * sin(theta);
	pIdq->Q = FIXP30_mpy(Iab.B, cosSin->cos) - FIXP30_mpy(Iab.A, cosSin->sin);

	return;
}

/*
 * 反Park变换
 * */
void Inv_Park_Duty( Duty_Ddq_t Ddq, FIXP_CosSin_t* cosSin, Duty_Dab_t * pDab)
{
	pDab->A = FIXP30_mpy(Ddq.D, cosSin->cos) - FIXP30_mpy(Ddq.Q, cosSin->sin);
	pDab->B = FIXP30_mpy(Ddq.Q, cosSin->cos) + FIXP30_mpy(Ddq.D, cosSin->sin);
	return;
}

/*
 * Clarke变换
 * r、s、t -> a、b
 * */
void Clarke_Current( const Currents_Irst_t Irst, Currents_Iab_t *pIab)
{
	/* Test: invert measured current vector polarity to check dq sign consistency. */
	pIab->A = -(FIXP30_mpy(Irst.R, FIXP30(2.0f / 3.0f)) - FIXP30_mpy(Irst.S + Irst.T, FIXP30(1.0f / 3.0f)));
	pIab->B = -FIXP30_mpy((Irst.S - Irst.T), FIXP30(0.577350269));

	return;
}

/*
 * 反Clarke变换
 * */
void Inv_Clarke_Duty(Duty_Dab_t dutyDab, Duty_Drst_t *pDutyDrst)
{
	// D_factor_a = -Dalpha/2
	fixp30_t D_factor_a = -(dutyDab.A >> 1);

	// D_factor_b = sqrt(3)/2 * Dbeta
	fixp30_t D_factor_b = FIXP30_mpy(FIXP30(SQRT_3_OVER_2), dutyDab.B);

	pDutyDrst->R = dutyDab.A;
	pDutyDrst->S = D_factor_a + D_factor_b;
	pDutyDrst->T = D_factor_a - D_factor_b;

	return;
}


/*
 * @brief 空间矢量调制，均值钳位SPWM，是SVPWM的等效简化算法
 * */
void Modulate(const Duty_Drst_t *pDutyDrstInput, Duty_Drst_t *pDutyDrstOutput)
{
	fixp30_t Dr, Ds, Dt, Dmin, Dmax, Dcom, De;

	Dr = pDutyDrstInput->R;
	Ds = pDutyDrstInput->S;
	Dt = pDutyDrstInput->T;

	// 找出最小值和最大值
	Dmin = (Dr > Ds ? Ds : Dr);
	Dmin = (Dmin > Dt ? Dt : Dmin);
	Dmax = (Dr < Ds ? Ds : Dr);
	Dmax = (Dmax < Dt ? Dt : Dmax);

	// 计算共模钳位电压，计算最大最小值的中点
	Dcom = (Dmin + Dmax) >> 1;

	// 三相全部减去这个共模量
	pDutyDrstOutput->R = Dr - Dcom;
	pDutyDrstOutput->S = Ds - Dcom;
	pDutyDrstOutput->T = Dt - Dcom;

	return;
}
