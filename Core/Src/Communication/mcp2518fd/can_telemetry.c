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

typedef enum
{
    CAN_FC_RSP_CMD_STATUS   = 0x20U,
    CAN_FC_RSP_PARAM_STATE  = 0x21U,
    CAN_FC_RSP_PARAM_RESULT1 = 0x22U,
    CAN_FC_RSP_PARAM_RESULT2 = 0x23U,
    CAN_FC_RSP_PARAM_DEBUG1 = 0x24U,
    CAN_FC_RSP_PARAM_DEBUG2 = 0x25U,
#if APP_USE_CAN_FD
    CAN_FC_TELEM_FOC        = 0x30U,
    CAN_FC_TELEM_STATUS     = 0x31U,
    CAN_FC_TELEM_SPEED_PWR  = 0x32U,
    CAN_FC_TELEM_TEMP       = 0x33U
#else
    CAN_FC_TELEM_STATUS     = 0x30U,
    CAN_FC_TELEM_CUR_REF    = 0x31U,
    CAN_FC_TELEM_CUR_CALC   = 0x32U,
    CAN_FC_TELEM_SPEED      = 0x33U,
    CAN_FC_TELEM_VBUS_TEMP  = 0x34U,
    CAN_FC_TELEM_TEMP       = 0x35U
#endif
} CanTelemetryFuncCode_t;

#define CAN_TX_QUEUE_DEPTH      16U

#define CAN_TELEM_PERIOD_STATUS_MS      2000U
#define CAN_TELEM_PERIOD_CUR_REF_MS     1000U
#define CAN_TELEM_PERIOD_CUR_CALC_MS    1000U
#define CAN_TELEM_PERIOD_SPEED_MS       1000U
#define CAN_TELEM_PERIOD_VBUS_TEMP_MS   5000U
#define CAN_TELEM_PERIOD_TEMP_MS       10000U

typedef struct
{
    uint16_t sid;
    uint8_t len;
    uint8_t data[APP_CAN_MAX_DATA_BYTES];
} CAN_Telemetry_Frame_t;

static CAN_Telemetry_Frame_t s_tx_queue[CAN_TX_QUEUE_DEPTH];
static uint8_t s_tx_queue_head = 0U;
static uint8_t s_tx_queue_tail = 0U;
static uint8_t s_tx_queue_count = 0U;

typedef struct
{
    CanTelemetryFuncCode_t funcCode;
    uint16_t period_ms;
    uint16_t elapsed_ms;
} CAN_Telemetry_Slot_t;

static CAN_Telemetry_Slot_t s_telem_slots[] =
{
#if APP_USE_CAN_FD
    {CAN_FC_TELEM_FOC,       CAN_TELEM_PERIOD_CUR_REF_MS,   0U},
    {CAN_FC_TELEM_SPEED_PWR, CAN_TELEM_PERIOD_SPEED_MS,     0U},
    {CAN_FC_TELEM_STATUS,    CAN_TELEM_PERIOD_STATUS_MS,    0U},
    {CAN_FC_TELEM_TEMP,      CAN_TELEM_PERIOD_VBUS_TEMP_MS, 0U}
#else
    {CAN_FC_TELEM_CUR_REF,   CAN_TELEM_PERIOD_CUR_REF_MS,   0U},
    {CAN_FC_TELEM_CUR_CALC,  CAN_TELEM_PERIOD_CUR_CALC_MS,  0U},
    {CAN_FC_TELEM_SPEED,     CAN_TELEM_PERIOD_SPEED_MS,     0U},
    {CAN_FC_TELEM_STATUS,    CAN_TELEM_PERIOD_STATUS_MS,    0U},
    {CAN_FC_TELEM_VBUS_TEMP, CAN_TELEM_PERIOD_VBUS_TEMP_MS, 0U},
    {CAN_FC_TELEM_TEMP,      CAN_TELEM_PERIOD_TEMP_MS,      0U}
#endif
};

static void CAN_Telemetry_SendFrame(uint16_t sid, const uint8_t *payload, uint8_t len)
{
    uint8_t txbuf[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t copy_len = (len > APP_CAN_MAX_DATA_BYTES) ? APP_CAN_MAX_DATA_BYTES : len;

    if (g_system_comm_mode != COMM_PROTO_CAN)
    {
        return;
    }

    (void)memcpy(txbuf, payload, copy_len);
    MCP2518FD_TransmitMessageQueue(DRV_CANFDSPI_INDEX_0, sid, txbuf, DRV_CANFDSPI_DataBytesToDlc(copy_len));
}

static uint16_t CAN_Telemetry_BuildId(CanTelemetryFuncCode_t funcCode, uint8_t nodeId)
{
    return CAN_MAKE_ID(funcCode, nodeId);
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
    uint8_t stateFrame[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t resultFrame1[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t resultFrame2[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t debugFrame1[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t debugFrame2[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t nodeId = ParamId_GetCanNodeId();
    uint8_t stateLen = 8U;
    uint8_t resultLen = 8U;
    ParamIdDebugData_t debugData;

    memset(&debugData, 0, sizeof(debugData));
    ParamId_GetDebugData(&debugData);

    stateFrame[0] = (uint8_t)g_axis.state;
    stateFrame[1] = paramState;
    stateFrame[2] = polePairs;
    stateFrame[3] = CAN_Telemetry_ParamValidBits(result);
    CAN_Telemetry_PackU16(&stateFrame[4], (uint16_t)polePairs);
#if APP_USE_CAN_FD
    if (APP_CAN_MAX_DATA_BYTES >= 12U)
    {
        stateFrame[6] = nodeId;
        stateFrame[7] = CAN_NODE_ID_MASK;
        stateFrame[8] = APP_CAN_FRAME_FDF;
        stateFrame[9] = APP_CAN_FRAME_BRS;
        stateLen = 12U;
    }
#endif
    (void)CAN_Telemetry_EnqueueFrame(CAN_Telemetry_BuildId(CAN_FC_RSP_PARAM_STATE, nodeId), stateFrame, stateLen);

    if (result != NULL)
    {
        CAN_Telemetry_PackFloat(&resultFrame1[0], result->rs_ohm);
        CAN_Telemetry_PackFloat(&resultFrame1[4], result->ld_h);
#if APP_USE_CAN_FD
        if (APP_CAN_MAX_DATA_BYTES >= 16U)
        {
            CAN_Telemetry_PackFloat(&resultFrame1[8], result->lq_h);
            CAN_Telemetry_PackFloat(&resultFrame1[12], result->ke_v_per_rad_s);
            resultLen = 16U;
        }
        else
#endif
        {
            CAN_Telemetry_PackFloat(&resultFrame2[0], result->lq_h);
            CAN_Telemetry_PackFloat(&resultFrame2[4], result->ke_v_per_rad_s);
        }
        (void)CAN_Telemetry_EnqueueFrame(CAN_Telemetry_BuildId(CAN_FC_RSP_PARAM_RESULT1, nodeId), resultFrame1, resultLen);
#if !APP_USE_CAN_FD
        (void)CAN_Telemetry_EnqueueFrame(CAN_Telemetry_BuildId(CAN_FC_RSP_PARAM_RESULT2, nodeId), resultFrame2, resultLen);
#endif
    }

    CAN_Telemetry_PackFloat(&debugFrame1[0], debugData.i_avg_a);
    CAN_Telemetry_PackFloat(&debugFrame1[4], debugData.v_avg_v);
    (void)CAN_Telemetry_EnqueueFrame(CAN_Telemetry_BuildId(CAN_FC_RSP_PARAM_DEBUG1, nodeId), debugFrame1, 8U);

    CAN_Telemetry_PackFloat(&debugFrame2[0], debugData.calc_id_a);
    CAN_Telemetry_PackFloat(&debugFrame2[4], debugData.calc_iq_a);
    (void)CAN_Telemetry_EnqueueFrame(CAN_Telemetry_BuildId(CAN_FC_RSP_PARAM_DEBUG2, nodeId), debugFrame2, 8U);
}

void CAN_Telemetry_Init(void)
{
    memset(&s_adc_shadow, 0, sizeof(s_adc_shadow));
    s_tx_queue_head = 0U;
    s_tx_queue_tail = 0U;
    s_tx_queue_count = 0U;
    memset(s_telem_slots, 0, sizeof(s_telem_slots));
#if APP_USE_CAN_FD
    s_telem_slots[0].funcCode = CAN_FC_TELEM_FOC;
    s_telem_slots[0].period_ms = CAN_TELEM_PERIOD_CUR_REF_MS;
    s_telem_slots[1].funcCode = CAN_FC_TELEM_SPEED_PWR;
    s_telem_slots[1].period_ms = CAN_TELEM_PERIOD_SPEED_MS;
    s_telem_slots[2].funcCode = CAN_FC_TELEM_STATUS;
    s_telem_slots[2].period_ms = CAN_TELEM_PERIOD_STATUS_MS;
    s_telem_slots[3].funcCode = CAN_FC_TELEM_TEMP;
    s_telem_slots[3].period_ms = CAN_TELEM_PERIOD_VBUS_TEMP_MS;
#else
    s_telem_slots[0].funcCode = CAN_FC_TELEM_CUR_REF;
    s_telem_slots[0].period_ms = CAN_TELEM_PERIOD_CUR_REF_MS;
    s_telem_slots[1].funcCode = CAN_FC_TELEM_CUR_CALC;
    s_telem_slots[1].period_ms = CAN_TELEM_PERIOD_CUR_CALC_MS;
    s_telem_slots[2].funcCode = CAN_FC_TELEM_SPEED;
    s_telem_slots[2].period_ms = CAN_TELEM_PERIOD_SPEED_MS;
    s_telem_slots[3].funcCode = CAN_FC_TELEM_STATUS;
    s_telem_slots[3].period_ms = CAN_TELEM_PERIOD_STATUS_MS;
    s_telem_slots[4].funcCode = CAN_FC_TELEM_VBUS_TEMP;
    s_telem_slots[4].period_ms = CAN_TELEM_PERIOD_VBUS_TEMP_MS;
    s_telem_slots[5].funcCode = CAN_FC_TELEM_TEMP;
    s_telem_slots[5].period_ms = CAN_TELEM_PERIOD_TEMP_MS;
#endif
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
    uint8_t copy_len = (len > APP_CAN_MAX_DATA_BYTES) ? APP_CAN_MAX_DATA_BYTES : len;

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

static uint8_t CAN_Telemetry_BuildCmdStatusPayload(uint16_t cmdSid,
                                                   uint8_t status,
                                                   uint8_t extra,
                                                   uint8_t rxLen,
                                                   uint8_t *payload)
{
    uint8_t payloadLen = 8U;

    if (payload == NULL)
    {
        return 0U;
    }

    memset(payload, 0, APP_CAN_MAX_DATA_BYTES);
    CAN_Telemetry_PackU16(&payload[0], cmdSid);
    payload[2] = status;
    payload[3] = (uint8_t)g_axis.state;
    payload[4] = extra;
    payload[5] = (uint8_t)MC_Calib_GetParamState();
    payload[6] = rxLen;
    payload[7] = MC_Get_Pole_Pairs();
#if APP_USE_CAN_FD
    if (APP_CAN_MAX_DATA_BYTES >= 12U)
    {
        payload[8] = ParamId_GetCanNodeId();
        payload[9] = s_adc_shadow.raw_COMM_ID;
        CAN_Telemetry_PackU16(&payload[10], s_adc_shadow.raw_TSENA);
        payloadLen = 12U;
    }
#endif
    return payloadLen;
}

void CAN_Telemetry_QueueCmdStatus(uint16_t cmdSid, uint8_t status, uint8_t extra, uint8_t rxLen, uint8_t nodeId)
{
    uint8_t payload[APP_CAN_MAX_DATA_BYTES];
    uint8_t payloadLen = CAN_Telemetry_BuildCmdStatusPayload(cmdSid, status, extra, rxLen, payload);

    if (payloadLen == 0U)
    {
        return;
    }

    (void)CAN_Telemetry_EnqueueFrame(CAN_Telemetry_BuildId(CAN_FC_RSP_CMD_STATUS, nodeId), payload, payloadLen);
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

static bool CAN_Telemetry_BuildPeriodicFrame(CanTelemetryFuncCode_t funcCode,
                                             uint8_t nodeId,
                                             uint16_t *sidOut,
                                             uint8_t *frame,
                                             uint8_t *lenOut)
{
    if ((sidOut == NULL) || (frame == NULL) || (lenOut == NULL))
    {
        return false;
    }

    memset(frame, 0, APP_CAN_MAX_DATA_BYTES);
    *sidOut = CAN_Telemetry_BuildId(funcCode, nodeId);

    switch (funcCode)
    {
#if APP_USE_CAN_FD
        case CAN_FC_TELEM_FOC:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.currCtrl.refIdq.D) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.currCtrl.refIdq.Q) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[8], FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[12], MotorControl_GetIqFilteredDisplayA());
            *lenOut = 16U;
            return true;

        case CAN_FC_TELEM_STATUS:
            frame[0] = (uint8_t)g_axis.state;
            frame[1] = (uint8_t)g_axis.error;
            frame[2] = (uint8_t)g_axis.enCtrlMode;
            frame[3] = s_adc_shadow.raw_COMM_ID;
            CAN_Telemetry_PackU16(&frame[4], s_adc_shadow.raw_TSENA);
            frame[6] = nodeId;
            frame[7] = MC_Get_Pole_Pairs();
            *lenOut = 8U;
            return true;

        case CAN_FC_TELEM_SPEED_PWR:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.speedCtrl.speedRef_pu) * FREQUENCY_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) * FREQUENCY_SCALE);
            CAN_Telemetry_PackFloat(&frame[8], FIXP30_toF(g_axis.busVoltage) * VOLTAGE_SCALE);
            CAN_Telemetry_PackFloat(&frame[12], s_adc_shadow.temp_TSENB_c);
            *lenOut = 16U;
            return true;

        case CAN_FC_TELEM_TEMP:
            CAN_Telemetry_PackFloat(&frame[0], s_adc_shadow.temp_TSENC_c);
            CAN_Telemetry_PackFloat(&frame[4], s_adc_shadow.temp_TSENA_c);
            CAN_Telemetry_PackFloat(&frame[8], s_adc_shadow.temp_TSENB_c);
            *lenOut = 12U;
            return true;
#else
        case CAN_FC_TELEM_STATUS:
            frame[0] = (uint8_t)g_axis.state;
            frame[1] = (uint8_t)g_axis.error;
            frame[2] = (uint8_t)g_axis.enCtrlMode;
            frame[3] = s_adc_shadow.raw_COMM_ID;
            CAN_Telemetry_PackU16(&frame[4], s_adc_shadow.raw_TSENA);
            *lenOut = 6U;
#if APP_USE_CAN_FD
            if (APP_CAN_MAX_DATA_BYTES >= 8U)
            {
                frame[6] = nodeId;
                frame[7] = (uint8_t)g_axis.error;
                *lenOut = 8U;
            }
#endif
            return true;

        case CAN_FC_TELEM_CUR_REF:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.currCtrl.refIdq.D) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.currCtrl.refIdq.Q) * CURRENT_SCALE);
            *lenOut = 8U;
#if APP_USE_CAN_FD
            if (APP_CAN_MAX_DATA_BYTES >= 16U)
            {
                CAN_Telemetry_PackFloat(&frame[8], FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE);
                CAN_Telemetry_PackFloat(&frame[12], MotorControl_GetIqFilteredDisplayA());
                *lenOut = 16U;
            }
#endif
            return true;

        case CAN_FC_TELEM_CUR_CALC:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.currCtrl.calcIdq.D) * CURRENT_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], MotorControl_GetIqFilteredDisplayA());
            *lenOut = 8U;
            return true;

        case CAN_FC_TELEM_SPEED:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.speedCtrl.speedRef_pu) * FREQUENCY_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], FIXP30_toF(g_axis.speedCtrl.speedMeas_pu) * FREQUENCY_SCALE);
            *lenOut = 8U;
            return true;

        case CAN_FC_TELEM_VBUS_TEMP:
            CAN_Telemetry_PackFloat(&frame[0], FIXP30_toF(g_axis.busVoltage) * VOLTAGE_SCALE);
            CAN_Telemetry_PackFloat(&frame[4], s_adc_shadow.temp_TSENB_c);
            *lenOut = 8U;
#if APP_USE_CAN_FD
            if (APP_CAN_MAX_DATA_BYTES >= 16U)
            {
                CAN_Telemetry_PackFloat(&frame[8], s_adc_shadow.temp_TSENC_c);
                CAN_Telemetry_PackFloat(&frame[12], s_adc_shadow.temp_TSENA_c);
                *lenOut = 16U;
            }
#endif
            return true;

        case CAN_FC_TELEM_TEMP:
            CAN_Telemetry_PackFloat(&frame[0], s_adc_shadow.temp_TSENC_c);
            CAN_Telemetry_PackFloat(&frame[4], s_adc_shadow.temp_TSENA_c);
            *lenOut = 8U;
            return true;
#endif

        default:
            return false;
    }
}

void CAN_Telemetry_Service1ms(void)
{
    CAN_Telemetry_Frame_t frameOut;
    uint8_t frame[APP_CAN_MAX_DATA_BYTES] = {0};
    uint8_t nodeId = ParamId_GetCanNodeId();
    uint16_t sid = 0U;
    uint8_t len = 0U;
    uint32_t i;

    if (g_system_comm_mode != COMM_PROTO_CAN)
    {
        return;
    }

    if (CAN_Telemetry_DequeueFrame(&frameOut))
    {
        CAN_Telemetry_SendFrame(frameOut.sid, frameOut.data, frameOut.len);
        return;
    }

    for (i = 0U; i < (sizeof(s_telem_slots) / sizeof(s_telem_slots[0])); i++)
    {
        if (s_telem_slots[i].elapsed_ms < s_telem_slots[i].period_ms)
        {
            s_telem_slots[i].elapsed_ms++;
        }
    }

    for (i = 0U; i < (sizeof(s_telem_slots) / sizeof(s_telem_slots[0])); i++)
    {
        if (s_telem_slots[i].elapsed_ms < s_telem_slots[i].period_ms)
        {
            continue;
        }

        if (CAN_Telemetry_BuildPeriodicFrame(s_telem_slots[i].funcCode, nodeId, &sid, frame, &len))
        {
            s_telem_slots[i].elapsed_ms = 0U;
            CAN_Telemetry_SendFrame(sid, frame, len);
            return;
        }
    }
}
