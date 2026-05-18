#include "tem_task.h"

#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "Communication/mcp2518fd/can_telemetry.h"

extern ADC_HandleTypeDef hadc2;

void Tem_Task(void *argument)
{
  (void)argument;

  uint8_t comm_id_locked = 0U;

  for (;;)
  {
    ADC_Rule_Collect(&hadc2, &g_adc_Rule_ID_Tem);
    CAN_Telemetry_UpdateFromAdc(&g_adc_Rule_ID_Tem);

    if (comm_id_locked == 0U)
    {
      g_system_comm_mode = (g_adc_Rule_ID_Tem.raw_COMM_ID != 0U) ? COMM_PROTO_ETHERCAT : COMM_PROTO_CAN;
      comm_id_locked = 1U;
    }

    osDelay(1000);
  }
}
