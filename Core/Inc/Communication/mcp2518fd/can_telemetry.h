/*
 * can_telemetry.h
 *
 *  Created on: May 18, 2026
 */

#ifndef INC_COMMUNICATION_MCP2518FD_CAN_TELEMETRY_H_
#define INC_COMMUNICATION_MCP2518FD_CAN_TELEMETRY_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void CAN_Telemetry_Init(void);
void CAN_Telemetry_UpdateFromAdc(const ADC_Rule_Data_t *pAdcData);
void CAN_Telemetry_Service1ms(void);
bool CAN_Telemetry_EnqueueFrame(uint16_t sid, const uint8_t *payload, uint8_t len);
void CAN_Telemetry_QueueCmdStatus(uint16_t cmdSid, uint8_t status, uint8_t extra, uint8_t rxLen, uint8_t nodeId);
void CAN_Telemetry_RequestRuntimeParamSnapshot(void);
bool CAN_Telemetry_RequestFlashParamSnapshot(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMUNICATION_MCP2518FD_CAN_TELEMETRY_H_ */
