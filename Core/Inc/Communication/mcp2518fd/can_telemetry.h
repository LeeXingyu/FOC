/*
 * can_telemetry.h
 *
 *  Created on: May 18, 2026
 */

#ifndef INC_COMMUNICATION_MCP2518FD_CAN_TELEMETRY_H_
#define INC_COMMUNICATION_MCP2518FD_CAN_TELEMETRY_H_

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void CAN_Telemetry_Init(void);
void CAN_Telemetry_UpdateFromAdc(const ADC_Rule_Data_t *pAdcData);
void CAN_Telemetry_Service1ms(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMUNICATION_MCP2518FD_CAN_TELEMETRY_H_ */
