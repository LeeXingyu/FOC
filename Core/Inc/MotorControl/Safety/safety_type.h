/*
 * safety_type.h
 *
 *  Created on: Apr 14, 2026
 *      Author: Administrator
 */

#ifndef INC_MOTORCONTROL_SAFETY_SAFETY_TYPE_H_
#define INC_MOTORCONTROL_SAFETY_SAFETY_TYPE_H_

typedef struct {
    /* ── 母线电压保护（V）── */
    float   fVbusOvervoltage;       // 过压阈值，
    float   fVbusUndervoltage;      // 欠压阈值
    float   fVbusRated;             // 额定电压

    /* ── 电流保护（A）── */
    float   fCurrentMax;            // 软件过流阈值
    float   fCurrentRated;          // 额定电流

    /* ── 温度保护（℃）── */
    float   fTempWarning;           // 过温警告阈值
    float   fTempShutdown;          // 过温关断阈值
} SafetyConfig_t;


#endif /* INC_MOTORCONTROL_SAFETY_SAFETY_TYPE_H_ */
