#ifndef INC_MOTORCONTROL_CORE_MC_INTERFACE_EXT_H_
#define INC_MOTORCONTROL_CORE_MC_INTERFACE_EXT_H_

#include <stdint.h>
#include <stdbool.h>

bool MC_Cia402Ext_ReadObject(uint16_t index, uint8_t subIndex,
                             uint8_t *value, uint8_t *size);
bool MC_Cia402Ext_WriteObject(uint16_t index, uint8_t subIndex,
                              const uint8_t *value, uint8_t size);

#endif
