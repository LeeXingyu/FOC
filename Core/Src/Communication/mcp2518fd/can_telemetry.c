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
#include "mc_tasks.h"
#include "param_identify.h"
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

#define CAN_RSP_CMD_STATUS      0x181U
#define CAN_RSP_PARAM_STATE     0x182U
#define CAN_RSP_PARAM_RESULT_1  0x183U
#define CAN_RSP_PARAM_RESULT_2  0x184U

#define CAN_TX_QUEUE_DEPTH      16U

#define CAN_TELEMETRY_PERIOD_MS  100U

typedef struct
{
    uint16_t sid;
    uint8_t len;
    uint8_t data[8];
} CAN_Telemetry_Frame_t;

static CAN_Telemetry_Frame_t s_tx_queue[CAN_TX_QUEUE_DEPTH];
static uint8_t s_tx_queue_head = 0U;
static uint8_t s_tx_queue_tail = 0U;
static uint8_t s_tx_queue_count = 0U;

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

static uint8_t CAN_Telemetry_ParamValidBits(const ParamIdResult_t *r)
{
    uint8_t bits = 0U;

    if (r == NULL)
    {
        return 0U;
    }

    if (r->validRs) { bits |= 0x01U; }
    if (r->validLd) { bits |= 0x02U; }
    if (r->validLq) { bits |= 0x04U; }
    if (r->validKe) { bits |= 0x08U; }
    return bits;
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

static bool CAN_Telemetry_DequeueFrame(CAN_Telemetry_Frame_t *frame)
{
    if ((frame == NULL) || (s_tx_queue_count == 0U))
    {
        return false;
    }

    *frame = s_tx_queue[s_tx_queue_head];
    s_tx_queue_head++;
    if (s_tx_queue_head >= CAN_TX_QUEUE_DEPTH)
    {
        s_tx_queue_head = 0U;
    }
    s_tx_queue_count--;
    return true;
}

static void CAN_Telemetry_QueueParamSnapshotCommon(const ParamIdResult_t *result,
                                                   uint8_t paramState,
                                                   uint8_t polePairs)
{
    uint8_t stateFrame[8] = {0};
    uint8_t resultFrame1[8] = {0};
    uint8_t resultFrame2[8] = {0};

    stateFrame[0] = (uint8_t)g_axis.state;
    stateFrame[1] = paramState;
    stateFrame[2] = polePairs;
    stateFrame[3] = CAN_Telemetry_ParamValidBits(result);
    CAN_Telemetry_PackU16(&stateFrame[4], (uint16_t)polePairs);
    (void)CAN_Telemetry_EnqueueFrame(CAN_RSP_PARAM_STATE, stateFrame, 8U);

    if (result != NULL)
    {
        CAN_Telemetry_PackFloat(&resultFrame1[0], result->rs_ohm);
        CAN_Telemetry_PackFloat(&resultFrame1[4], result->ld_h);
        CAN_Telemetry_PackFloat(&resultFrame2[0], result->lq_h);
        CAN_Telemetry_PackFloat(&resultFrame2[4], result->ke_v_per_rad_s);
        (void)CAN_Telemetry_EnqueueFrame(CAN_RSP_PARAM_RESULT_1, resultFrame1, 8U);
        (void)CAN_Telemetry_EnqueueFrame(CAN_RSP_PARAM_RESULT_2, resultFrame2, 8U);
    }
}

void CAN_Telemetry_Init(void)
{
    memset(&s_adc_shadow, 0, sizeof(s_adc_shadow));
    s_tick_div = 0U;
    s_frame_index = 0U;
    s_tx_queue_head = 0U;
    s_tx_queue_tail = 0U;
    s_tx_queue_count = 0U;
}

void CAN_Telemetry_UpdateFromAdc(const ADC_Rule_Data_t *pAdcData)
{
    if (pAdcData != NULL)
    {
        s_adc_shadow = *pAdcData;
    }
}

bool CAN_Telemetry_EnqueueFrame(uint16_t sid, const uint8_t *payload, uint8_t len)
{
    CAN_Telemetry_Frame_t *slot;
    uint8_t copy_len = (len > 8U) ? 8U : len;

    if (s_tx_queue_count >= CAN_TX_QUEUE_DEPTH)
    {
        return false;
    }

    slot = &s_tx_queue[s_tx_queue_tail];
    slot->sid = sid;
    slot->len = copy_len;
    memset(slot->data, 0, sizeof(slot->data));
    if ((payload != NULL) && (copy_len > 0U))
    {
        (void)memcpy(slot->data, payload, copy_len);
    }

    s_tx_queue_tail++;
    if (s_tx_queue_tail >= CAN_TX_QUEUE_DEPTH)
    {
        s_tx_queue_tail = 0U;
    }
    s_tx_queue_count++;
    return true;
}

void CAN_Telemetry_QueueCmdStatus(uint16_t cmdSid, uint8_t status, uint8_t extra, uint8_t rxLen)
{
    uint8_t payload[8] = {0};

    CAN_Telemetry_PackU16(&payload[0], cmdSid);
    payload[2] = status;
    payload[3] = (uint8_t)g_axis.state;
    payload[4] = extra;
    payload[5] = (uint8_t)MC_Calib_GetParamState();
    payload[6] = rxLen;
    payload[7] = MC_Get_Pole_Pairs();
    (void)CAN_Telemetry_EnqueueFrame(CAN_RSP_CMD_STATUS, payload, 8U);
}

void CAN_Telemetry_RequestRuntimeParamSnapshot(void)
{
    CAN_Telemetry_QueueParamSnapshotCommon(MC_Calib_GetParamResult(),
                                          (uint8_t)MC_Calib_GetParamState(),
                                          MC_Get_Pole_Pairs());
}

bool CAN_Telemetry_RequestFlashParamSnapshot(void)
{
    ParamIdFlashData_t data;

    if (!ParamId_LoadFromFlash(&data))
    {
        return false;
    }

    CAN_Telemetry_QueueParamSnapshotCommon(&data.result,
                                          (uint8_t)PARAM_ID_STATE_DONE,
                                          (uint8_t)data.pole_pairs);
    return true;
}

void CAN_Telemetry_Service1ms(void)
{
    CAN_Telemetry_Frame_t frameOut;
    uint8_t frame[8] = {0};

    if (g_system_comm_mode != COMM_PROTO_CAN)
    {
        return;
    }

    if (CAN_Telemetry_DequeueFrame(&frameOut))
    {
        CAN_Telemetry_SendFrame(frameOut.sid, frameOut.data, frameOut.len);
        return;
    }

    s_tick_div++;
    if (s_tick_div < CAN_TELEMETRY_PERIOD_MS)
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
