#ifndef INC_COMMUNICATION_CDC_DEBUG_H_
#define INC_COMMUNICATION_CDC_DEBUG_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t axis_state;
    uint8_t axis_error;
    uint8_t control_mode;
    uint8_t param_state;
    float position_deg;
    float position_cont_deg;
    float position_mech_deg;
    float position_app_pu;
    float position_app_deg;
    float speed_rpm;
    float speed_meas_rpm;
    float speed_error_rpm;
    float speed_meas_pu;
    float speed_error_pu;
    float current_d_a;
    float current_q_a;
    float current_ref_q_a;
} CDC_DebugTelemetry_t;

void CDC_Debug_Init(void);
void CDC_Debug_Service(void);
void CDC_Debug_TaskHook(void);
void CDC_Debug_SetTelemetry(const CDC_DebugTelemetry_t *telemetry);
bool CDC_Debug_Enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMUNICATION_CDC_DEBUG_H_ */
