/*
 * can_telemetry.c
 *
 * CAN 上发上位机的遥测层：
 * 1) 采集源分离：FOC 状态来自电机控制数据，温度来自 ADC 采集线程。
 * 2) 分帧发送：每帧 8 字节，避免一帧塞太多内容。
 * 3) 周期轮询：1ms 服务一次，按固定顺序轮发，降低总线抖动。
 * 4) 模式门控：仅在 CAN 模式下发送，ETHERCAT 模式保持空实现。
 */

#include "can_telemetry.h"
#include "canopen.h"
#include "mc_interface.h"
#include "curr_fbdk.h"
#include "speed_pos_fbdk.h"
#include "motor_control.h"
#include "motor_parameters.h"
#include "cmsis_os.h"
#include <string.h>

typedef union
{
    float f;
    uint8_t b[4];
} FloatBytes_t;

static ADC_Rule_Data_t s_adc_shadow;
static uint8_t s_tick_div = 0U;
static uint8_t s_frame_index = 0U;

static void CAN_Telemetry_SendFrame(uint16_t sid, const uint8_t *payload, uint8_t len)
{
    uint8_t txbuf[8] = {0};
    uint8_t copy_len = (len > 8U) ? 8U : len;

    if (g_system_comm_mode != COMM_PROTO_CAN)
    {
        return;
    }

    (void)memcpy(txbuf, payload, copy_len);
    MCP2518FD_TransmitMessageQueue(DRV_CANFDSPI_INDEX_0, sid, txbuf, DRV_CANFDSPI_DataBytesToDlc(copy_len));
}

static void CAN_Telemetry_PackU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void CAN_Telemetry_PackFloat(uint8_t *dst, float value)
{
    FloatBytes_t u;
    u.f = value;
    dst[0] = u.b[0];
    dst[1] = u.b[1];
    dst[2] = u.b[2];
    dst[3] = u.b[3];
}

void CAN_Telemetry_Init(void)
{
    memset(&s_adc_shadow, 0, sizeof(s_adc_shadow));
    s_tick_div = 0U;
    s_frame_index = 0U;
}

void CAN_Telemetry_UpdateFromAdc(const ADC_Rule_Data_t *pAdcData)
{
    if (pAdcData != NULL)
    {
        s_adc_shadow = *pAdcData;
    }
}

void CAN_Telemetry_Service1ms(void)
{
    uint8_t frame[8] = {0};

    if (g_system_comm_mode != COMM_PROTO_CAN)
    {
        return;
    }

    s_tick_div++;
    if (s_tick_div < 1U)
    {
        return;
    }
    s_tick_div = 0U;

    switch (s_frame_index)
    {
        case 0U:
            frame[0] = (uint8_t)g_axis.state;
            frame[1] = (uint8_t)g_axis.error;
            frame[2] = (uint8_t)g_axis.enCtrlMode;
            frame[3] = s_adc_shadow.raw_COMM_ID;
            CAN_Telemetry_PackU16(&frame[4], s_adc_shadow.raw_TSENA);
            CAN_Telemetry_SendFrame(0x201U, frame, 6U);
            break;

        case 1U:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.currCtrl.refIdq.D) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.currCtrl.refIdq.Q) * CURRENT_SCALE);
            CAN_Telemetry_SendFrame(0x202U, frame, 8U);
            break;

        case 2U:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.currCtrl.calcIdq.Q) * CURRENT_SCALE);
            CAN_Telemetry_SendFrame(0x203U, frame, 8U);
            break;

        case 3U:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.speedCtrl.speedRef_pu) * FREQUENCY_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) * FREQUENCY_SCALE);
            CAN_Telemetry_SendFrame(0x204U, frame, 8U);
            break;

        case 4U:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.busVoltage) * VOLTAGE_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], s_adc_shadow.temp_TSENB_c);
            CAN_Telemetry_SendFrame(0x205U, frame, 8U);
            break;

        case 5U:
            CAN_Telemetry_PackFloat(&frame[0], s_adc_shadow.temp_TSENC_c);
            CAN_Telemetry_PackFloat(&frame[4], s_adc_shadow.temp_TSENA_c);
            CAN_Telemetry_SendFrame(0x206U, frame, 8U);
            break;

        default:
            s_frame_index = 0U;
            return;
    }

    s_frame_index++;
    if (s_frame_index >= 6U)
    {
        s_frame_index = 0U;
    }
}
