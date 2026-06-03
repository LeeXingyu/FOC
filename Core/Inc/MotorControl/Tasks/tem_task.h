#ifndef INC_MOTORCONTROL_TASKS_TEM_TASK_H_
#define INC_MOTORCONTROL_TASKS_TEM_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define Q24_BASE                (16777216L) // 2^24
#define ADC12_MAX_COUNTS        (4095U)
#define NTC_PULLUP_OHMS         (10000.0f)  // 上拉电阻 10k
#define KELVIN_TO_CELSIUS_Q24   ((int32_t)(273.15f * (float)Q24_BASE))
#define SH_A_Q24                (10637L)
#define SH_B_Q24                (4963L)
#define SH_C_Q24                (2L)

void Tem_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTORCONTROL_TASKS_TEM_TASK_H_ */
