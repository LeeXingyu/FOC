/*
 * ethercat.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_COMMUNICATION_ETHERCAT_H_
#define INC_COMMUNICATION_ETHERCAT_H_

#include <stdint.h>

typedef struct
{
    uint16_t controlword;
    uint16_t statusword;
    uint8_t mode_of_operation;
    uint8_t mode_display;
    uint8_t axis_state;
    uint8_t axis_error;
} EtherCAT_Cia402Shadow_t;

void LAN9253_Init(void);
void LAN9253_Process(void);
void LAN9253_SetTaskHandle(void *taskHandle);
const EtherCAT_Cia402Shadow_t *LAN9253_GetCia402Shadow(void);


#endif /* INC_COMMUNICATION_ETHERCAT_H_ */
