#include "tem_task.h"

#include "adc.h"
#include "cmsis_os.h"
#include "main.h"
#include "can_telemetry.h"
#include <math.h>

extern ADC_HandleTypeDef hadc2;


static int32_t Q24_Mul(int32_t a, int32_t b)
{
	 return (int32_t)((((int64_t)a * (int64_t)b) + (Q24_BASE >> 1)) / Q24_BASE);
}

float ADC_ThermistorToCelsius_Q24(uint16_t adc_raw)
{
    float adc_f = 0.0f;
    float resistance_ohms = 0.0f;
    int32_t ln_r_q24 = 0;
    int32_t ln_r2_q24 = 0;
    int32_t ln_r3_q24 = 0;
    int32_t inv_t_q24 = 0;
    int32_t temp_k_q24 = 0;
    int32_t temp_c_q24 = 0;

    // 1. 边界检查
    if ((adc_raw == 0U) || (adc_raw >= ADC12_MAX_COUNTS))
    {
        return 0.0f; // 或者返回错误代码
    }

    // 2. 计算电阻值 (使用浮点计算电阻，因为涉及除法，定点数容易溢出且没必要)
    adc_f = (float)adc_raw;
    // 公式: R_ntc = R_pullup * ADC / (4095 - ADC)
    resistance_ohms = (NTC_PULLUP_OHMS * adc_f) / ((float)ADC12_MAX_COUNTS - adc_f);

    if (resistance_ohms <= 0.0f) return 0.0f;

    // 3. 计算 ln(R) 并转换为 Q24
    // 注意：logf 返回 float，乘以 Q24_BASE 转为定点数
    ln_r_q24 = (int32_t)(logf(resistance_ohms) * (float)Q24_BASE);

    // 4. 计算 ln(R)^2 和 ln(R)^3
    ln_r2_q24 = Q24_Mul(ln_r_q24, ln_r_q24);
    ln_r3_q24 = Q24_Mul(ln_r2_q24, ln_r_q24);

    // 5. Steinhart-Hart 方程: 1/T = A + B*lnR + C*(lnR)^3
    inv_t_q24 = SH_A_Q24;
    inv_t_q24 += Q24_Mul(SH_B_Q24, ln_r_q24);
    inv_t_q24 += Q24_Mul(SH_C_Q24, ln_r3_q24);

    // 6. 防止除以 0 或溢出
    if (inv_t_q24 < 20000L || inv_t_q24 > 100000L)
    {
        return 0.0f;
    }

    // 7. 计算 T(K) = 1 / (1/T)
    // 因为 inv_t_q24 是 Q24 格式 (即 真实值 * 2^24)
    // 所以 1/inv_t_q24 的真实值是 1 / (inv_t_q24 / 2^24) = 2^24 / inv_t_q24
    // 我们需要结果也是 Q24 格式，所以要再乘 2^24
    // 结果 = (2^24 * 2^24) / inv_t_q24
    float inv_t_float = (float)inv_t_q24 / (float)Q24_BASE;
    // 2. 计算 T(K)
    float temp_k_float = 1.0f / inv_t_float;
    // 3. 转为摄氏度
    float temp_c_float = temp_k_float - 273.15f;

    return temp_c_float;
}

void Tem_Task(void *argument)
{
  (void)argument;

  uint8_t comm_id_locked = 0U;

  for (;;)
  {
    ADC_Rule_Collect(&hadc2, &g_adc_Rule_ID_Tem);
    g_adc_Rule_ID_Tem.temp_TSENA_c = ADC_ThermistorToCelsius_Q24(g_adc_Rule_ID_Tem.raw_TSENA);
    g_adc_Rule_ID_Tem.temp_TSENB_c = ADC_ThermistorToCelsius_Q24(g_adc_Rule_ID_Tem.raw_TSENB);
    g_adc_Rule_ID_Tem.temp_TSENC_c = ADC_ThermistorToCelsius_Q24(g_adc_Rule_ID_Tem.raw_TSENC);
    CAN_Telemetry_UpdateFromAdc(&g_adc_Rule_ID_Tem);

    if (comm_id_locked == 0U)
    {
      g_system_comm_mode = (g_adc_Rule_ID_Tem.raw_COMM_ID == 1U) ? COMM_PROTO_CAN : COMM_PROTO_ETHERCAT;
      comm_id_locked = 1U;
    }

    osDelay(3000);
  }
}
