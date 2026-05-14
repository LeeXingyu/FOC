/*
 * mc_math.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_UTILS_MC_MATH_H_
#define INC_UTILS_MC_MATH_H_

/*
 * 卡尔曼滤波结构体
 * */
typedef struct
{
    float fOptimalEstimateValue;  	// 最优估计值
    float fErrorCovariance;  		// 误差协方差
    float fMeasurementNoise;  		// 测量噪声
    float fProcessNoise;  			// 过程噪声
} KalmanFilter_t;

extern KalmanFilter_t g_motorSpeedKalmanFilter;
//extern KalmanFilter_t g_idKalmanFilter;
//extern KalmanFilter_t g_iqFalmanFilter;

/*
 * 卡尔曼滤波初始化
 * */
void Kalman_Filter_Init(KalmanFilter_t *kf, float r, float q);

void Set_Kalman_Filter_RQ(KalmanFilter_t *kf, float r, float q);

/*
 * 卡尔曼滤波计算
 * */
float Kalman_Filter_Calc(KalmanFilter_t *kf, float input);





#endif /* INC_UTILS_MC_MATH_H_ */
