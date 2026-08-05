/*
 * pid_position.h
 */

#ifndef INC_UTILS_PID_PID_POSITION_H_
#define INC_UTILS_PID_PID_POSITION_H_

#include <stdint.h>
#include <stdbool.h>
#include "mc_type.h"
#include "pid_position_type.h"

float Pid_Position_Run(PosCtrl_t *pPosCtrl);

#endif /* INC_UTILS_PID_PID_POSITION_H_ */
