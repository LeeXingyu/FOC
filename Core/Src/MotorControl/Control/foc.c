/*
 * foc.c
 */

#include "foc.h"

#define SQRT_3_OVER_2 (0.86602540378f)

void Park_Current(const Currents_Iab_t Iab, FIXP_CosSin_t *cosSin, Currents_Idq_t *pIdq)
{
	pIdq->D = FIXP30_mpy(Iab.A, cosSin->cos) + FIXP30_mpy(Iab.B, cosSin->sin);
	pIdq->Q = FIXP30_mpy(Iab.B, cosSin->cos) - FIXP30_mpy(Iab.A, cosSin->sin);
}

void Inv_Park_Duty(Duty_Ddq_t Ddq, FIXP_CosSin_t *cosSin, Duty_Dab_t *pDab)
{
	pDab->A = FIXP30_mpy(Ddq.D, cosSin->cos) - FIXP30_mpy(Ddq.Q, cosSin->sin);
	pDab->B = FIXP30_mpy(Ddq.Q, cosSin->cos) + FIXP30_mpy(Ddq.D, cosSin->sin);
}

void Clarke_Current(const Currents_Irst_t Irst, Currents_Iab_t *pIab)
{
	pIab->A = FIXP30_mpy(Irst.R, FIXP30(2.0f / 3.0f)) -
	          FIXP30_mpy(Irst.S + Irst.T, FIXP30(1.0f / 3.0f));
	pIab->B = FIXP30_mpy((Irst.S - Irst.T), FIXP30(0.577350269f));
}

void Inv_Clarke_Duty(Duty_Dab_t dutyDab, Duty_Drst_t *pDutyDrst)
{
	fixp30_t D_factor_a = -(dutyDab.A >> 1);
	fixp30_t D_factor_b = FIXP30_mpy(FIXP30(SQRT_3_OVER_2), dutyDab.B);

	pDutyDrst->R = dutyDab.A;
	pDutyDrst->S = D_factor_a + D_factor_b;
	pDutyDrst->T = D_factor_a - D_factor_b;
}

void Modulate(const Duty_Drst_t *pDutyDrstInput, Duty_Drst_t *pDutyDrstOutput)
{
	fixp30_t Dr = pDutyDrstInput->R;
	fixp30_t Ds = pDutyDrstInput->S;
	fixp30_t Dt = pDutyDrstInput->T;
	fixp30_t Dmin = (Dr < Ds) ? Dr : Ds;
	fixp30_t Dmax = (Dr > Ds) ? Dr : Ds;
	fixp30_t Dcom;

	Dmin = (Dmin < Dt) ? Dmin : Dt;
	Dmax = (Dmax > Dt) ? Dmax : Dt;
	Dcom = (Dmin + Dmax) >> 1;

	pDutyDrstOutput->R = Dr - Dcom;
	pDutyDrstOutput->S = Ds - Dcom;
	pDutyDrstOutput->T = Dt - Dcom;
}
