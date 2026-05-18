/*
 * mc_math.c
 *
 *  Created on: Apr 9, 2026
 *      Author: xzh
 *  des: Park / Clarke /
 */

#include "mc_math.h"

KalmanFilter_t g_motorSpeedKalmanFilter;

/*
 * 卡尔曼滤波初始化
 * */
void Kalman_Filter_Init(KalmanFilter_t *kf, float r, float q)
{
    kf->fOptimalEstimateValue = 0;
    kf->fErrorCovariance = 1;
    kf->fMeasurementNoise = r;
    kf->fProcessNoise = q;
}

void Set_Kalman_Filter_RQ(KalmanFilter_t *kf, float r, float q)
{
    kf->fMeasurementNoise = r;
    kf->fProcessNoise = q;
}

/*
 * 卡尔曼滤波计算
 * 当前逻辑中的唯一核心作用是：抑制速度测量噪声，稳定速度误差，从而让 iq 给定更平稳。
 * */
float Kalman_Filter_Calc(KalmanFilter_t *kf, float input)
{
    kf->fErrorCovariance = kf->fErrorCovariance + kf->fProcessNoise;
    float g = kf->fErrorCovariance / (kf->fErrorCovariance + kf->fMeasurementNoise);
    kf->fOptimalEstimateValue = kf->fOptimalEstimateValue + g * (input - kf->fOptimalEstimateValue);
    kf->fErrorCovariance = (1 - g) * kf->fErrorCovariance;
    return kf->fOptimalEstimateValue;
}
