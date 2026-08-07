#include "canopen.h"
#include "can_telemetry.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "param_identify.h"
#include "motor_parameters.h"

#include <math.h>

static MCP2518FD_Status_t s_mcp2518_status = {0};
static ParamIdState_t s_param_state_prev = PARAM_ID_STATE_IDLE;

#define CAN_CMD_SPEED_ABS_LIMIT_RPM  1000.0f
#define CAN_CMD_GAIN_MAX             1000.0f
#define CAN_TX_WAIT_TIMEOUT_MS       5U
#define CAN_SPI_TRANSFER_TIMEOUT_MS   2U
#define CANOPEN_HEARTBEAT_DEFAULT_MS  1000U
#define CANOPEN_TPDO_PERIOD_MS        20U
#define CANOPEN_SDO_MAX_DATA          4U
#define CANOPEN_SDO_SEGMENT_BUFFER    255U

typedef enum
{
    CANOPEN_NMT_INITIALIZING = 0U,
    CANOPEN_NMT_STOPPED      = 4U,
    CANOPEN_NMT_OPERATIONAL  = 5U,
    CANOPEN_NMT_PREOP        = 127U
} CanOpenNmtState_t;

static CanOpenNmtState_t s_canopen_nmt_state = CANOPEN_NMT_INITIALIZING;
static uint16_t s_canopen_heartbeat_ms = CANOPEN_HEARTBEAT_DEFAULT_MS;
static uint32_t s_canopen_heartbeat_elapsed_ms = 0U;
static uint32_t s_canopen_tpdo_elapsed_ms = 0U;
static uint32_t s_canopen_tpdo2_elapsed_ms = 0U;
static uint8_t s_canopen_rpdo1_map_count = 2U;
static uint8_t s_canopen_tpdo1_map_count = 4U;
static uint8_t s_canopen_tpdo1_transmission = 0xFFU;
static uint8_t s_canopen_tpdo2_transmission = 0xFFU;
static uint8_t s_canopen_tpdo2_map_count = 2U;
static AxisError_t s_canopen_last_error = AXIS_ERROR_NONE;
static uint8_t s_canopen_sync_counter = 0U;
static uint32_t s_canopen_rpdo1_mapping[2] = {0x60400010UL, 0x60FF0020UL};
static uint32_t s_canopen_tpdo1_mapping[4] = {
    0x60410010UL, 0x606C0020UL, 0x60610008UL, 0x10010008UL
};
static uint32_t s_canopen_rpdo2_cobid;
static uint32_t s_canopen_tpdo2_cobid;
static uint8_t s_canopen_rpdo2_transmission = 0xFFU;
static uint8_t s_canopen_tpdo2_enabled = 1U;
static uint16_t s_canopen_tpdo2_inhibit_ms = 0U;
static uint16_t s_canopen_tpdo2_event_ms = CANOPEN_TPDO_PERIOD_MS;
static uint8_t s_canopen_rpdo2_map_count = 2U;
static uint32_t s_canopen_rpdo2_mapping[2] = {0x60600008UL, 0x60710010UL};
static uint32_t s_canopen_tpdo2_mapping[2] = {0x60640020UL, 0x607A0020UL};
static uint32_t s_canopen_rpdo1_cobid;
static uint32_t s_canopen_tpdo1_cobid;
static uint8_t s_canopen_rpdo1_transmission = 0xFFU;
static uint8_t s_canopen_tpdo1_enabled = 1U;
static uint16_t s_canopen_tpdo1_inhibit_ms = 0U;
static uint16_t s_canopen_tpdo1_event_ms = CANOPEN_TPDO_PERIOD_MS;

typedef struct
{
    uint8_t active;
    uint8_t toggle;
    uint16_t index;
    uint8_t subIndex;
    uint16_t length;
    uint16_t offset;
    uint8_t data[CANOPEN_SDO_SEGMENT_BUFFER];
} CanOpenSdoSession_t;

static CanOpenSdoSession_t s_canopen_sdo_upload;
static CanOpenSdoSession_t s_canopen_sdo_download;

static void CanOpen_Send(uint16_t cobId, const uint8_t *data, uint8_t len)
{
    uint8_t payload[APP_CAN_MAX_DATA_BYTES] = {0};

    if (len > APP_CAN_MAX_DATA_BYTES)
    {
        len = APP_CAN_MAX_DATA_BYTES;
    }
    (void)memcpy(payload, data, len);
    MCP2518FD_TransmitMessageQueue(DRV_CANFDSPI_INDEX_0, cobId, payload,
                                   DRV_CANFDSPI_DataBytesToDlc(len));
}

static void CanOpen_SendHeartbeat(void)
{
    uint8_t state = (uint8_t)s_canopen_nmt_state;
    CanOpen_Send((uint16_t)(CANOPEN_COBID_HEARTBEAT_BASE + ParamId_GetCanNodeId()),
                 &state, 1U);
}

static void CanOpen_SendBootup(void)
{
    uint8_t bootup = 0U;

    CanOpen_Send((uint16_t)(CANOPEN_COBID_HEARTBEAT_BASE +
                            ParamId_GetCanNodeId()),
                 &bootup, 1U);
}

static void CanOpen_SendEmcy(AxisError_t error)
{
    uint8_t payload[8] = {0};
    uint16_t errorCode = (error == AXIS_ERROR_NONE) ? 0U : 0x2310U;

    payload[0] = (uint8_t)errorCode;
    payload[1] = (uint8_t)(errorCode >> 8);
    payload[2] = (error == AXIS_ERROR_NONE) ? 0U : 1U;
    payload[3] = (uint8_t)error;
    CanOpen_Send((uint16_t)(0x080U + ParamId_GetCanNodeId()), payload, 8U);
}

static void CanOpen_SendTpdo(void)
{
    uint8_t payload[8] = {0};
    uint8_t offset = 0U;
    uint8_t i;

    for (i = 0U; i < s_canopen_tpdo1_map_count; i++)
    {
        uint16_t index = (uint16_t)(s_canopen_tpdo1_mapping[i] >> 16);
        uint8_t subIndex = (uint8_t)(s_canopen_tpdo1_mapping[i] >> 8);
        uint8_t bits = (uint8_t)s_canopen_tpdo1_mapping[i];
        uint8_t value[4] = {0};
        uint8_t valueSize = 0U;

        if ((bits == 0U) || ((bits % 8U) != 0U) ||
            !MC_Cia402_ReadObject(index, subIndex, value, &valueSize) ||
            (valueSize != (uint8_t)(bits / 8U)) ||
            ((uint8_t)(offset + valueSize) > sizeof(payload)))
        {
            return;
        }
        (void)memcpy(&payload[offset], value, valueSize);
        offset = (uint8_t)(offset + valueSize);
    }
    if (s_canopen_tpdo1_enabled != 0U)
    {
        CanOpen_Send((uint16_t)(s_canopen_tpdo1_cobid & 0x7FFU),
                     payload, offset);
    }
}

static void CanOpen_SendTpdo2(void)
{
    uint8_t payload[8] = {0};
    uint8_t offset = 0U;
    uint8_t i;

    if (s_canopen_tpdo2_enabled == 0U)
    {
        return;
    }
    for (i = 0U; i < s_canopen_tpdo2_map_count; i++)
    {
        uint16_t index = (uint16_t)(s_canopen_tpdo2_mapping[i] >> 16);
        uint8_t subIndex = (uint8_t)(s_canopen_tpdo2_mapping[i] >> 8);
        uint8_t bits = (uint8_t)s_canopen_tpdo2_mapping[i];
        uint8_t value[4] = {0};
        uint8_t valueSize = 0U;

        if ((bits == 0U) || ((bits % 8U) != 0U) ||
            !MC_Cia402_ReadObject(index, subIndex, value, &valueSize) ||
            (valueSize != (uint8_t)(bits / 8U)) ||
            ((uint8_t)(offset + valueSize) > sizeof(payload)))
        {
            return;
        }
        (void)memcpy(&payload[offset], value, valueSize);
        offset = (uint8_t)(offset + valueSize);
    }
    CanOpen_Send((uint16_t)(s_canopen_tpdo2_cobid & 0x7FFU),
                 payload, offset);
}

static void CanOpen_SendSdoAbort(uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, uint32_t abortCode)
{
    uint8_t response[8] = {
        0x80U, (uint8_t)(index & 0xFFU), (uint8_t)(index >> 8),
        subIndex, (uint8_t)(abortCode & 0xFFU),
        (uint8_t)(abortCode >> 8), (uint8_t)(abortCode >> 16),
        (uint8_t)(abortCode >> 24)
    };
    CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId), response, 8U);
}

static uint32_t CanOpen_GetWriteAbortCode(uint16_t index)
{
    return ((index == 0x6040U) ||
            (index == 0x6060U) ||
            (index == 0x60FFU) ||
            (index == 0x6071U) ||
            (index == 0x607AU)) ?
           0x06090030UL : 0x06010002UL;
}

static bool CanOpen_GetWritableObjectSize(uint16_t index, uint8_t subIndex,
                                          uint8_t *size)
{
    if (size == NULL)
    {
        return false;
    }

    switch (index)
    {
        case 0x6040U:
        case 0x6071U:
            if (subIndex == 0U)
            {
                *size = 2U;
                return true;
            }
            break;
        case 0x6060U:
            if (subIndex == 0U)
            {
                *size = 1U;
                return true;
            }
            break;
        case 0x60FFU:
        case 0x607AU:
            if (subIndex == 0U)
            {
                *size = 4U;
                return true;
            }
            break;
        case 0x1017U:
            if (subIndex == 0U)
            {
                *size = 2U;
                return true;
            }
            break;
        case 0x1400U:
        case 0x1401U:
        case 0x1800U:
        case 0x1801U:
            if (subIndex == 1U)
            {
                *size = 4U;
                return true;
            }
            if (subIndex == 2U)
            {
                *size = 1U;
                return true;
            }
            if ((index == 0x1800U || index == 0x1801U) &&
                ((subIndex == 3U) || (subIndex == 5U)))
            {
                *size = 2U;
                return true;
            }
            break;
        case 0x1600U:
        case 0x1601U:
        case 0x1A00U:
        case 0x1A01U:
            if (subIndex == 0U)
            {
                *size = 1U;
                return true;
            }
            if (((index == 0x1600U) || (index == 0x1601U)) &&
                (subIndex <= 2U))
            {
                *size = 4U;
                return true;
            }
            if ((index == 0x1A00U) && (subIndex <= 4U))
            {
                *size = 4U;
                return true;
            }
            if ((index == 0x1A01U) && (subIndex <= 2U))
            {
                *size = 4U;
                return true;
            }
            break;
        default:
            break;
    }

    return false;
}

static bool CanOpen_IsValidCobId(uint32_t cobId)
{
    /*
     * Only the standard 11-bit COB-ID and the CANopen disable bit are
     * supported.  Bits 11..30 are reserved and must not leak into a
     * transmitted standard CAN identifier.
     */
    return ((cobId & 0x7FFFF800UL) == 0U) &&
           ((cobId & 0x7FFU) != 0U);
}

static bool CanOpen_IsAllowedTxCobId(uint16_t cobId)
{
#if APP_USE_LEGACY_CAN_PROTOCOL
    (void)cobId;
    return true;
#else
    uint16_t nodeId = ParamId_GetCanNodeId();

    /*
     * Strict builds expose only the CANopen producer objects.  The TPDO
     * entries are compared against their configured COB-ID so a standard
     * master may remap them through 0x1800/0x1801.
     */
    return (cobId == (uint16_t)(0x080U + nodeId)) ||
           (cobId == (uint16_t)(s_canopen_tpdo1_cobid & 0x7FFU)) ||
           (cobId == (uint16_t)(s_canopen_tpdo2_cobid & 0x7FFU)) ||
           (cobId == (uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId)) ||
           (cobId == (uint16_t)(CANOPEN_COBID_HEARTBEAT_BASE + nodeId));
#endif
}

static void CanOpen_SendSdoValue(uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, const uint8_t *value,
                                 uint8_t size)
{
    uint8_t response[8] = {0};
    uint8_t command = (size == 1U) ? 0x4FU : (size == 2U) ? 0x4BU : 0x43U;

    if ((size == 0U) || (size > CANOPEN_SDO_MAX_DATA))
    {
        return;
    }
    response[0] = command;
    response[1] = (uint8_t)(index & 0xFFU);
    response[2] = (uint8_t)(index >> 8);
    response[3] = subIndex;
    (void)memcpy(&response[4], value, size);
    CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId), response, 8U);
}

static void CanOpen_SendSdoSegment(uint8_t nodeId)
{
    uint8_t response[8] = {0};
    uint16_t remaining = (uint16_t)(s_canopen_sdo_upload.length -
                                    s_canopen_sdo_upload.offset);
    uint8_t count = (remaining > 7U) ? 7U : (uint8_t)remaining;
    uint8_t unused = (uint8_t)(7U - count);
    uint8_t command = (uint8_t)(s_canopen_sdo_upload.toggle << 4);

    if (count == remaining)
    {
        command |= (uint8_t)(unused << 1);
        command |= 1U;
    }
    (void)memcpy(&response[1],
                 &s_canopen_sdo_upload.data[s_canopen_sdo_upload.offset],
                 count);
    response[0] = command;
    s_canopen_sdo_upload.offset = (uint16_t)(s_canopen_sdo_upload.offset + count);
    s_canopen_sdo_upload.toggle ^= 1U;
    if (count == remaining)
    {
        s_canopen_sdo_upload.active = 0U;
    }
    CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId), response, 8U);
}

static bool CanOpen_ReadObject(uint16_t index, uint8_t subIndex,
                               uint8_t *value, uint8_t *size)
{
    uint32_t u32;

    if ((index == 0x6040U) || (index == 0x6041U) ||
        (index == 0x6060U) || (index == 0x6061U) ||
        (index == 0x60FFU) || (index == 0x606CU) ||
        (index == 0x6071U) || (index == 0x607AU) ||
        (index == 0x6064U))
    {
        return MC_Cia402_ReadObject(index, subIndex, value, size);
    }

    if ((subIndex != 0U) &&
        (index != 0x1018U) && (index != 0x1400U) &&
        (index != 0x1401U) && (index != 0x1600U) &&
        (index != 0x1601U) && (index != 0x1800U) &&
        (index != 0x1801U) && (index != 0x1A00U) &&
        (index != 0x1A01U))
    {
        return false;
    }
    switch (index)
    {
        case 0x1000U:
            u32 = 0x00020192UL;
            (void)memcpy(value, &u32, sizeof(u32));
            *size = 4U;
            return true;
        case 0x1001U:
            value[0] = (g_axis.error != AXIS_ERROR_NONE) ? 1U : 0U;
            *size = 1U;
            return true;
        case 0x1008U:
        {
            static const uint8_t deviceName[] = "LX BLDC CiA402";
            (void)memcpy(value, deviceName, sizeof(deviceName) - 1U);
            *size = (uint8_t)(sizeof(deviceName) - 1U);
            return true;
        }
        case 0x1017U:
            value[0] = (uint8_t)s_canopen_heartbeat_ms;
            value[1] = (uint8_t)(s_canopen_heartbeat_ms >> 8);
            *size = 2U;
            return true;
        case 0x1018U:
            if (subIndex == 0U)
            {
                value[0] = 4U;
            }
            else if (subIndex == 1U)
            {
                u32 = 0x00000001UL;
                (void)memcpy(value, &u32, sizeof(u32));
            }
            else if (subIndex == 2U)
            {
                u32 = 0x00000192UL;
                (void)memcpy(value, &u32, sizeof(u32));
            }
            else if (subIndex == 3U)
            {
                u32 = 0x00000001UL;
                (void)memcpy(value, &u32, sizeof(u32));
            }
            else if (subIndex == 4U)
            {
                u32 = 0x00000000UL;
                (void)memcpy(value, &u32, sizeof(u32));
            }
            else
            {
                return false;
            }
            *size = (subIndex == 0U) ? 1U : 4U;
            return true;
        case 0x1400U:
            if (subIndex == 0U)
            {
                value[0] = 2U;
                *size = 1U;
            }
            else if (subIndex == 1U)
            {
                u32 = s_canopen_rpdo1_cobid;
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 2U)
            {
                value[0] = s_canopen_rpdo1_transmission;
                *size = 1U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1600U:
            if (subIndex == 0U)
            {
                value[0] = s_canopen_rpdo1_map_count;
                *size = 1U;
            }
            else if (subIndex == 1U)
            {
                u32 = s_canopen_rpdo1_mapping[0];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 2U)
            {
                u32 = s_canopen_rpdo1_mapping[1];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1401U:
            if (subIndex == 0U)
            {
                value[0] = 2U;
                *size = 1U;
            }
            else if (subIndex == 1U)
            {
                u32 = s_canopen_rpdo2_cobid;
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 2U)
            {
                value[0] = s_canopen_rpdo2_transmission;
                *size = 1U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1601U:
            if (subIndex == 0U)
            {
                value[0] = s_canopen_rpdo2_map_count;
                *size = 1U;
            }
            else if ((subIndex >= 1U) && (subIndex <= 2U))
            {
                u32 = s_canopen_rpdo2_mapping[subIndex - 1U];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1800U:
            if (subIndex == 0U)
            {
                value[0] = 5U;
                *size = 1U;
            }
            else if (subIndex == 1U)
            {
                u32 = s_canopen_tpdo1_cobid;
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 2U)
            {
                value[0] = s_canopen_tpdo1_transmission;
                *size = 1U;
            }
            else if (subIndex == 3U)
            {
                value[0] = (uint8_t)s_canopen_tpdo1_inhibit_ms;
                value[1] = (uint8_t)(s_canopen_tpdo1_inhibit_ms >> 8);
                *size = 2U;
            }
            else if (subIndex == 5U)
            {
                value[0] = (uint8_t)s_canopen_tpdo1_event_ms;
                value[1] = (uint8_t)(s_canopen_tpdo1_event_ms >> 8);
                *size = 2U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1801U:
            if (subIndex == 0U)
            {
                value[0] = 5U;
                *size = 1U;
            }
            else if (subIndex == 1U)
            {
                u32 = s_canopen_tpdo2_cobid;
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 2U)
            {
                value[0] = s_canopen_tpdo2_transmission;
                *size = 1U;
            }
            else if (subIndex == 3U)
            {
                value[0] = (uint8_t)s_canopen_tpdo2_inhibit_ms;
                value[1] = (uint8_t)(s_canopen_tpdo2_inhibit_ms >> 8);
                *size = 2U;
            }
            else if (subIndex == 5U)
            {
                value[0] = (uint8_t)s_canopen_tpdo2_event_ms;
                value[1] = (uint8_t)(s_canopen_tpdo2_event_ms >> 8);
                *size = 2U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1A01U:
            if (subIndex == 0U)
            {
                value[0] = s_canopen_tpdo2_map_count;
                *size = 1U;
            }
            else if ((subIndex >= 1U) && (subIndex <= 2U))
            {
                u32 = s_canopen_tpdo2_mapping[subIndex - 1U];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else
            {
                return false;
            }
            return true;
        case 0x1A00U:
            if (subIndex == 0U)
            {
                value[0] = s_canopen_tpdo1_map_count;
                *size = 1U;
            }
            else if (subIndex == 1U)
            {
                u32 = s_canopen_tpdo1_mapping[0];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 2U)
            {
                u32 = s_canopen_tpdo1_mapping[1];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 3U)
            {
                u32 = s_canopen_tpdo1_mapping[2];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else if (subIndex == 4U)
            {
                u32 = s_canopen_tpdo1_mapping[3];
                (void)memcpy(value, &u32, sizeof(u32));
                *size = 4U;
            }
            else
            {
                return false;
            }
            return true;
        default:
            return false;
    }
}

static bool CanOpen_WriteObject(uint16_t index, uint8_t subIndex,
                                const uint8_t *value, uint8_t size)
{
    uint32_t cobId;

    if ((index == 0x6040U) || (index == 0x6060U) ||
        (index == 0x60FFU) || (index == 0x6071U) ||
        (index == 0x607AU))
    {
        return MC_Cia402_WriteObject(index, subIndex, value, size);
    }

    if ((subIndex != 0U) &&
        (index != 0x1400U) && (index != 0x1600U) &&
        (index != 0x1401U) && (index != 0x1601U) &&
        (index != 0x1800U) && (index != 0x1801U) &&
        (index != 0x1A00U) && (index != 0x1A01U))
    {
        return false;
    }
    switch (index)
    {
        case 0x1017U:
            if (size != 2U)
            {
                return false;
            }
            s_canopen_heartbeat_ms = (uint16_t)value[0] | ((uint16_t)value[1] << 8);
            return true;
        case 0x1400U:
            if (subIndex == 1U && size == 4U)
            {
                (void)memcpy(&cobId, value, sizeof(cobId));
                if (!CanOpen_IsValidCobId(cobId))
                {
                    return false;
                }
                s_canopen_rpdo1_cobid = cobId;
                return true;
            }
            if (subIndex == 2U && size == 1U)
            {
                s_canopen_rpdo1_transmission = value[0];
                return true;
            }
            return false;
        case 0x1401U:
            if (subIndex == 1U && size == 4U)
            {
                (void)memcpy(&cobId, value, sizeof(cobId));
                if (!CanOpen_IsValidCobId(cobId))
                {
                    return false;
                }
                s_canopen_rpdo2_cobid = cobId;
                return true;
            }
            if (subIndex == 2U && size == 1U)
            {
                s_canopen_rpdo2_transmission = value[0];
                return true;
            }
            return false;
        case 0x1800U:
            if (subIndex == 1U && size == 4U)
            {
                (void)memcpy(&cobId, value, sizeof(cobId));
                if (!CanOpen_IsValidCobId(cobId))
                {
                    return false;
                }
                s_canopen_tpdo1_cobid = cobId;
                s_canopen_tpdo1_enabled =
                    ((s_canopen_tpdo1_cobid & 0x80000000UL) == 0U) ? 1U : 0U;
                return true;
            }
            if (subIndex == 2U && size == 1U)
            {
                s_canopen_tpdo1_transmission = value[0];
                return true;
            }
            if (subIndex == 3U && size == 2U)
            {
                s_canopen_tpdo1_inhibit_ms =
                    (uint16_t)value[0] | ((uint16_t)value[1] << 8);
                return true;
            }
            if (subIndex == 5U && size == 2U)
            {
                s_canopen_tpdo1_event_ms =
                    (uint16_t)value[0] | ((uint16_t)value[1] << 8);
                return true;
            }
            return false;
        case 0x1600U:
            if (subIndex == 0U)
            {
                if (size != 1U || value[0] > 2U)
                {
                    return false;
                }
                s_canopen_rpdo1_map_count = value[0];
                return true;
            }
            if ((subIndex <= 2U) && (size == 4U) && (subIndex >= 1U))
            {
                (void)memcpy(&s_canopen_rpdo1_mapping[subIndex - 1U],
                             value, sizeof(uint32_t));
                return true;
            }
            return false;
        case 0x1601U:
            if (subIndex == 0U)
            {
                if (size != 1U || value[0] > 2U)
                {
                    return false;
                }
                s_canopen_rpdo2_map_count = value[0];
                return true;
            }
            if ((subIndex >= 1U) && (subIndex <= 2U) && (size == 4U))
            {
                (void)memcpy(&s_canopen_rpdo2_mapping[subIndex - 1U],
                             value, 4U);
                return true;
            }
            return false;
        case 0x1A00U:
            if (subIndex == 0U)
            {
                if (size != 1U || value[0] > 4U)
                {
                    return false;
                }
                s_canopen_tpdo1_map_count = value[0];
                return true;
            }
            if ((subIndex <= 4U) && (size == 4U) && (subIndex >= 1U))
            {
                (void)memcpy(&s_canopen_tpdo1_mapping[subIndex - 1U],
                             value, sizeof(uint32_t));
                return true;
            }
            return false;
        case 0x1801U:
            if (subIndex == 1U && size == 4U)
            {
                (void)memcpy(&cobId, value, sizeof(cobId));
                if (!CanOpen_IsValidCobId(cobId))
                {
                    return false;
                }
                s_canopen_tpdo2_cobid = cobId;
                s_canopen_tpdo2_enabled =
                    ((s_canopen_tpdo2_cobid & 0x80000000UL) == 0U) ? 1U : 0U;
                return true;
            }
            if (subIndex == 2U && size == 1U)
            {
                s_canopen_tpdo2_transmission = value[0];
                return true;
            }
            if (subIndex == 3U && size == 2U)
            {
                s_canopen_tpdo2_inhibit_ms =
                    (uint16_t)value[0] | ((uint16_t)value[1] << 8);
                return true;
            }
            if (subIndex == 5U && size == 2U)
            {
                s_canopen_tpdo2_event_ms =
                    (uint16_t)value[0] | ((uint16_t)value[1] << 8);
                return true;
            }
            return false;
        case 0x1A01U:
            if (subIndex == 0U)
            {
                if (size != 1U || value[0] > 2U)
                {
                    return false;
                }
                s_canopen_tpdo2_map_count = value[0];
                return true;
            }
            if ((subIndex >= 1U) && (subIndex <= 2U) && (size == 4U))
            {
                (void)memcpy(&s_canopen_tpdo2_mapping[subIndex - 1U],
                             value, 4U);
                return true;
            }
            return false;
        default:
            return false;
    }
}

static bool CanOpen_HandleNmt(const CAN_RX_MSGOBJ *rxObj,
                              const uint8_t *data, uint8_t len)
{
    uint8_t command;
    uint8_t target;
    uint8_t nodeId = ParamId_GetCanNodeId();

    (void)rxObj;
    /*
     * Some PC CAN tools always transmit an 8-byte payload and pad the unused
     * bytes with zero. Standard NMT uses the first two bytes only, so accept
     * any frame that contains at least the command and node ID.
     */
    if (len < 2U)
    {
        return false;
    }
    command = data[0];
    target = data[1];
    if ((target != 0U) && (target != nodeId))
    {
        return true;
    }
    switch (command)
    {
        case 0x01U:
            s_canopen_nmt_state = CANOPEN_NMT_OPERATIONAL;
            break;
        case 0x02U:
            s_canopen_nmt_state = CANOPEN_NMT_STOPPED;
            (void)MC_Stop_Motor();
            break;
        case 0x80U:
            s_canopen_nmt_state = CANOPEN_NMT_PREOP;
            break;
        case 0x81U:
            MC_Cia402_ResetState();
            memset(&s_canopen_sdo_upload, 0, sizeof(s_canopen_sdo_upload));
            memset(&s_canopen_sdo_download, 0, sizeof(s_canopen_sdo_download));
            s_canopen_heartbeat_elapsed_ms = 0U;
            s_canopen_tpdo_elapsed_ms = 0U;
            s_canopen_tpdo2_elapsed_ms = 0U;
            s_canopen_nmt_state = CANOPEN_NMT_PREOP;
            CanOpen_SendBootup();
            break;
        default:
            return false;
    }
    return true;
}

static bool CanOpen_HandleSdo(const uint8_t *data, uint8_t len)
{
    uint8_t command;
    uint16_t index;
    uint8_t subIndex;
    uint8_t value[CANOPEN_SDO_SEGMENT_BUFFER] = {0};
    uint8_t size = 0U;
    uint8_t nodeId = ParamId_GetCanNodeId();

    if (len == 0U)
    {
        return false;
    }
    command = data[0];

    if (s_canopen_sdo_upload.active)
    {
        if ((len != 8U) || ((command & 0xEFU) != 0x60U))
        {
            CanOpen_SendSdoAbort(nodeId, s_canopen_sdo_upload.index,
                                 s_canopen_sdo_upload.subIndex, 0x05040001UL);
            s_canopen_sdo_upload.active = 0U;
            return true;
        }
        if (((command >> 4) & 1U) != s_canopen_sdo_upload.toggle)
        {
            CanOpen_SendSdoAbort(nodeId, s_canopen_sdo_upload.index,
                                 s_canopen_sdo_upload.subIndex, 0x05030000UL);
            s_canopen_sdo_upload.active = 0U;
            return true;
        }
        CanOpen_SendSdoSegment(nodeId);
        return true;
    }

    if (s_canopen_sdo_download.active)
    {
        uint8_t count;
        uint8_t unused;
        bool lastSegment;

        if ((command & 0xE0U) != 0x00U ||
            (len != 8U) ||
            ((command >> 4) & 1U) != s_canopen_sdo_download.toggle)
        {
            CanOpen_SendSdoAbort(nodeId, s_canopen_sdo_download.index,
                                 s_canopen_sdo_download.subIndex,
                                 0x05040001UL);
            s_canopen_sdo_download.active = 0U;
            return true;
        }
        unused = (uint8_t)((command >> 1) & 7U);
        lastSegment = ((command & 1U) != 0U);
        if ((!lastSegment && (unused != 0U)) ||
            (unused > 7U))
        {
            CanOpen_SendSdoAbort(nodeId, s_canopen_sdo_download.index,
                                 s_canopen_sdo_download.subIndex,
                                 0x05040001UL);
            s_canopen_sdo_download.active = 0U;
            return true;
        }
        count = (uint8_t)(7U - unused);
        if ((count > (uint8_t)(len - 1U)) ||
            ((uint16_t)(s_canopen_sdo_download.offset + count) >
             s_canopen_sdo_download.length))
        {
            CanOpen_SendSdoAbort(nodeId, s_canopen_sdo_download.index,
                                 s_canopen_sdo_download.subIndex,
                                 0x06070010UL);
            s_canopen_sdo_download.active = 0U;
            return true;
        }
        (void)memcpy(&s_canopen_sdo_download.data[s_canopen_sdo_download.offset],
                     &data[1], count);
        s_canopen_sdo_download.offset = (uint16_t)
            (s_canopen_sdo_download.offset + count);
        if (lastSegment)
        {
            bool ok = (s_canopen_sdo_download.offset ==
                       s_canopen_sdo_download.length) &&
                      CanOpen_WriteObject(s_canopen_sdo_download.index,
                                          s_canopen_sdo_download.subIndex,
                                          s_canopen_sdo_download.data,
                                          (uint8_t)s_canopen_sdo_download.length);
            s_canopen_sdo_download.active = 0U;
            if (!ok)
            {
                CanOpen_SendSdoAbort(nodeId, s_canopen_sdo_download.index,
                                     s_canopen_sdo_download.subIndex,
                                     (s_canopen_sdo_download.offset !=
                                      s_canopen_sdo_download.length) ?
                                     0x06070010UL :
                                     CanOpen_GetWriteAbortCode(
                                         s_canopen_sdo_download.index));
                return true;
            }
        }
        {
            uint8_t response[8] = {(uint8_t)(0x20U |
                (s_canopen_sdo_download.toggle << 4))};
            CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId),
                         response, 8U);
        }
        s_canopen_sdo_download.toggle ^= 1U;
        return true;
    }

    if (len < 4U)
    {
        CanOpen_SendSdoAbort(nodeId, 0U, 0U, 0x05040001UL);
        return true;
    }
    index = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
    subIndex = data[3];
    if (command == 0x40U)
    {
        if (len != 8U)
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x05040001UL);
            return true;
        }
        if (CanOpen_ReadObject(index, subIndex, value, &size))
        {
            if (size <= CANOPEN_SDO_MAX_DATA)
            {
                CanOpen_SendSdoValue(nodeId, index, subIndex, value, size);
            }
            else
            {
                uint8_t response[8] = {
                    0x41U, data[1], data[2], data[3],
                    size, 0U, 0U, 0U
                };
                (void)memcpy(s_canopen_sdo_upload.data, value, size);
                s_canopen_sdo_upload.active = 1U;
                s_canopen_sdo_upload.toggle = 0U;
                s_canopen_sdo_upload.index = index;
                s_canopen_sdo_upload.subIndex = subIndex;
                s_canopen_sdo_upload.length = size;
                s_canopen_sdo_upload.offset = 0U;
                response[5] = (uint8_t)(size >> 8);
                CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId),
                             response, 8U);
            }
        }
        else
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06020000UL);
        }
        return true;
    }
    if (command == 0x21U)
    {
        uint32_t downloadLength;
        uint8_t objectSize = 0U;

        if (len != 8U)
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06070010UL);
            return true;
        }
        downloadLength = (uint32_t)data[4] |
                         ((uint32_t)data[5] << 8) |
                         ((uint32_t)data[6] << 16) |
                         ((uint32_t)data[7] << 24);
        if ((downloadLength == 0U) ||
            (downloadLength > CANOPEN_SDO_SEGMENT_BUFFER))
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06070010UL);
            return true;
        }
        if (CanOpen_GetWritableObjectSize(index, subIndex, &objectSize) &&
            (downloadLength != objectSize))
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06070010UL);
            return true;
        }
        memset(&s_canopen_sdo_download, 0, sizeof(s_canopen_sdo_download));
        s_canopen_sdo_download.active = 1U;
        s_canopen_sdo_download.index = index;
        s_canopen_sdo_download.subIndex = subIndex;
        s_canopen_sdo_download.length = (uint16_t)downloadLength;
        {
            uint8_t response[8] = {0x60U, data[1], data[2], data[3]};
            CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId),
                         response, 8U);
        }
        return true;
    }
    if ((command != 0x2FU) && (command != 0x2BU) &&
        (command != 0x23U))
    {
        CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x05040001UL);
        return true;
    }
    size = (command == 0x2FU) ? 1U : (command == 0x2BU) ? 2U : 4U;
    if (len != 8U)
    {
        CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06070010UL);
    }
    else
    {
        uint8_t objectSize = 0U;

        if (!CanOpen_GetWritableObjectSize(index, subIndex, &objectSize))
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06010002UL);
        }
        else if (size != objectSize)
        {
            /*
             * Report a data-size mismatch before calling the object layer.
             * In particular, 0x6040 is UNSIGNED16 and must use 0x2B,
             * not a 4-byte 0x23 download command.
             */
            CanOpen_SendSdoAbort(nodeId, index, subIndex, 0x06070010UL);
        }
        else if (!CanOpen_WriteObject(index, subIndex, &data[4], size))
        {
            CanOpen_SendSdoAbort(nodeId, index, subIndex,
                                 CanOpen_GetWriteAbortCode(index));
        }
        else
        {
            uint8_t response[8] = {0x60U, data[1], data[2], data[3]};
            CanOpen_Send((uint16_t)(CANOPEN_COBID_SDO_TX_BASE + nodeId),
                         response, 8U);
        }
    }
    return true;
}

static bool CanOpen_HandleRpdo1(const uint8_t *data, uint8_t len)
{
    uint8_t offset = 0U;
    uint8_t i;

    for (i = 0U; i < s_canopen_rpdo1_map_count; i++)
    {
        uint16_t index = (uint16_t)(s_canopen_rpdo1_mapping[i] >> 16);
        uint8_t subIndex = (uint8_t)(s_canopen_rpdo1_mapping[i] >> 8);
        uint8_t bits = (uint8_t)s_canopen_rpdo1_mapping[i];
        uint8_t valueSize;

        if ((bits == 0U) || ((bits % 8U) != 0U))
        {
            return false;
        }
        valueSize = (uint8_t)(bits / 8U);
        if ((uint8_t)(offset + valueSize) > len)
        {
            return false;
        }
        if (!CanOpen_WriteObject(index, subIndex, &data[offset], valueSize))
        {
            return false;
        }
        offset = (uint8_t)(offset + valueSize);
    }
    return true;
}

static bool CanOpen_HandleRpdo2(const uint8_t *data, uint8_t len)
{
    uint8_t offset = 0U;
    uint8_t i;

    for (i = 0U; i < s_canopen_rpdo2_map_count; i++)
    {
        uint16_t index = (uint16_t)(s_canopen_rpdo2_mapping[i] >> 16);
        uint8_t subIndex = (uint8_t)(s_canopen_rpdo2_mapping[i] >> 8);
        uint8_t bits = (uint8_t)s_canopen_rpdo2_mapping[i];
        uint8_t valueSize;

        if ((bits == 0U) || ((bits % 8U) != 0U))
        {
            return false;
        }
        valueSize = (uint8_t)(bits / 8U);
        if ((uint8_t)(offset + valueSize) > len ||
            !CanOpen_WriteObject(index, subIndex, &data[offset], valueSize))
        {
            return false;
        }
        offset = (uint8_t)(offset + valueSize);
    }
    return true;
}

static bool CanOpen_HandleStandardFrame(uint16_t sid, const uint8_t *data,
                                        uint8_t len)
{
    uint8_t nodeId = ParamId_GetCanNodeId();

    if (sid == CANOPEN_COBID_NMT)
    {
        return CanOpen_HandleNmt(NULL, data, len);
    }
    if (sid == 0x080U)
    {
        if (len > 0U)
        {
            s_canopen_sync_counter = data[0];
        }
        else
        {
            s_canopen_sync_counter++;
        }
        if (s_canopen_nmt_state == CANOPEN_NMT_OPERATIONAL &&
            (s_canopen_tpdo1_transmission >= 1U) &&
            (s_canopen_tpdo1_transmission <= 240U))
        {
            CanOpen_SendTpdo();
        }
        if (s_canopen_nmt_state == CANOPEN_NMT_OPERATIONAL &&
            (s_canopen_tpdo2_transmission >= 1U) &&
            (s_canopen_tpdo2_transmission <= 240U))
        {
            CanOpen_SendTpdo2();
        }
        return true;
    }
    if (sid == (uint16_t)(CANOPEN_COBID_SDO_RX_BASE + nodeId))
    {
        return CanOpen_HandleSdo(data, len);
    }
    if (sid == (uint16_t)(s_canopen_rpdo1_cobid & 0x7FFU))
    {
        return ((s_canopen_rpdo1_cobid & 0x80000000UL) == 0U) &&
               (s_canopen_nmt_state == CANOPEN_NMT_OPERATIONAL) ?
               CanOpen_HandleRpdo1(data, len) : false;
    }
    if (sid == (uint16_t)(s_canopen_rpdo2_cobid & 0x7FFU))
    {
        return ((s_canopen_rpdo2_cobid & 0x80000000UL) == 0U) &&
               (s_canopen_nmt_state == CANOPEN_NMT_OPERATIONAL) ?
               CanOpen_HandleRpdo2(data, len) : false;
    }
    return false;
}

typedef enum
{
    CAN_FC_STOP_MOTOR       = 0x01U,
    CAN_FC_START_MOTOR      = 0x02U,
    CAN_FC_SET_MODE_SPEED   = 0x03U,
    CAN_FC_SET_MODE_POS     = 0x04U,
    CAN_FC_SET_MODE_VF      = 0x05U,
    CAN_FC_SET_REF_SPEED    = 0x06U,
    CAN_FC_SET_SPEED_KP     = 0x07U,
    CAN_FC_SET_SPEED_KI     = 0x08U,
    CAN_FC_SET_POLE_PAIRS   = 0x09U,
    CAN_FC_CALIB_START      = 0x0AU,
    CAN_FC_CALIB_STOP       = 0x0BU,
    CAN_FC_FLASH_READ_PARAM = 0x0CU,
    CAN_FC_FLASH_CLEAR      = 0x0DU,
    CAN_FC_GET_ID           = 0x0EU,
    CAN_FC_SET_ID           = 0x0FU,
    CAN_FC_CIA402_CONTROLWORD = 0x10U,
    CAN_FC_CIA402_MODE        = 0x11U,
    CAN_FC_CIA402_TARGET_SP   = 0x12U,
    CAN_FC_CIA402_TARGET_TQ   = 0x13U,
    CAN_FC_CIA402_STATUS      = 0x14U,
    CAN_FC_RSP_CMD_STATUS   = 0x20U,
    CAN_FC_RSP_PARAM_STATE  = 0x21U,
    CAN_FC_RSP_PARAM_RESULT1 = 0x22U,
    CAN_FC_RSP_PARAM_RESULT2 = 0x23U,
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
} CanFuncCode_t;

#define CAN_MODE_SWITCH_TIMEOUT_MS   100U

static bool Can_WaitOpMode(CAN_OPERATION_MODE mode)
{
    uint32_t startTick = HAL_GetTick();

    while (DRV_CANFDSPI_OperationModeGet(DRV_CANFDSPI_INDEX_0) != mode)
    {
        if ((HAL_GetTick() - startTick) >= CAN_MODE_SWITCH_TIMEOUT_MS)
        {
            return false;
        }
    }

    return true;
}

static bool Can_EnterOpMode(CAN_OPERATION_MODE mode)
{
    if (DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, mode) != 0)
    {
        return false;
    }

    return Can_WaitOpMode(mode);
}

static bool Can_ConfigureRxFilter(uint8_t nodeId)
{
    REG_CiFLTOBJ fObj;
    REG_CiMASK mObj;

    fObj.word = 0U;
    (void)nodeId;
    fObj.bF.SID = 0U;
    fObj.bF.SID11 = 0U;
    fObj.bF.EXIDE = 0U;
    fObj.bF.EID = 0U;
    if (DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF) != 0)
    {
        return false;
    }

    mObj.word = 0U;
    mObj.bF.MSID = 0U;
    mObj.bF.MSID11 = 0U;
    mObj.bF.MIDE = 1U;
    mObj.bF.MEID = 0U;
    if (DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF) != 0)
    {
        return false;
    }

    if (DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, CAN_FIFO_CH1, true) != 0)
    {
        return false;
    }

    return true;
}

static bool Can_ConfigureBroadcastGetIdFilter(void)
{
    REG_CiFLTOBJ fObj;
    REG_CiMASK mObj;
    uint16_t sid = CAN_MAKE_ID(CAN_FC_GET_ID, 0U);

    fObj.word = 0U;
    fObj.bF.SID = sid;
    fObj.bF.SID11 = 0U;
    fObj.bF.EXIDE = 0U;
    fObj.bF.EID = 0U;
    if (DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER1, &fObj.bF) != 0)
    {
        return false;
    }

    mObj.word = 0U;
    mObj.bF.MSID = 0x07FFU;
    mObj.bF.MSID11 = 0U;
    mObj.bF.MIDE = 1U;
    mObj.bF.MEID = 0U;
    if (DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER1, &mObj.bF) != 0)
    {
        return false;
    }

    if (DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER1, CAN_FIFO_CH1, true) != 0)
    {
        return false;
    }

    return true;
}

static bool Can_UpdateRxFilterForCurrentNode(void)
{
    bool ok;

    if (!Can_EnterOpMode(CAN_CONFIGURATION_MODE))
    {
        return false;
    }

    (void)DRV_CANFDSPI_FilterDisable(DRV_CANFDSPI_INDEX_0, CAN_FILTER0);
    ok = Can_ConfigureRxFilter(ParamId_GetCanNodeId());

    if (!Can_EnterOpMode(CAN_NORMAL_MODE))
    {
        return false;
    }

    return ok;
}

typedef enum
{
    CAN_CALIB_CHAIN_ALL = 0U,
    CAN_CALIB_PARAM_RS,
    CAN_CALIB_PARAM_LD,
    CAN_CALIB_PARAM_LQ,
    CAN_CALIB_PARAM_KE,
    CAN_CALIB_PARAM_ALL
} CanCalibCmd_t;

typedef enum
{
    CAN_CMD_STATUS_OK = 0U,
    CAN_CMD_STATUS_BAD_LEN,
    CAN_CMD_STATUS_BAD_ARG,
    CAN_CMD_STATUS_BAD_STATE,
    CAN_CMD_STATUS_BUSY,
    CAN_CMD_STATUS_UNKNOWN
} CanCmdStatus_t;

typedef CanCmdStatus_t (*CanCmdHandler)(const uint8_t *data, uint8_t len, uint8_t *extra);

typedef struct
{
    uint8_t funcCode;
    uint8_t expected_len;
    CanCmdHandler handler;
} CanCmdTable_t;

static float Can_ReadFloatLE(const uint8_t *data)
{
    union
    {
        float f;
        uint8_t b[4];
    } u = {0};

    u.b[0] = data[0];
    u.b[1] = data[1];
    u.b[2] = data[2];
    u.b[3] = data[3];
    return u.f;
}

static int32_t Can_ReadS32LE(const uint8_t *data)
{
    return (int32_t)(((uint32_t)data[0]) |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
}

static uint16_t Can_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

static int16_t Can_ReadS16LE(const uint8_t *data)
{
    return (int16_t)Can_ReadU16LE(data);
}

static void Can_SendCmdStatus(uint16_t cmdSid, CanCmdStatus_t status, uint8_t extra, uint8_t nodeId)
{
    CAN_Telemetry_QueueCmdStatus(cmdSid, (uint8_t)status, extra, s_mcp2518_status.last_rx_len, nodeId);
}

static CanCmdStatus_t Can_FlashClear(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;

    if (MC_Calib_GetParamState() == PARAM_ID_STATE_PREPARE ||
        MC_Calib_GetParamState() == PARAM_ID_STATE_LOCK_CHECK ||
        MC_Calib_GetParamState() == PARAM_ID_STATE_RUN)
    {
        return CAN_CMD_STATUS_BUSY;
    }

    return ParamId_ClearFlash() ? CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_STATE;
}

static CanCmdStatus_t Can_FlashReadParam(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;

    if (!CAN_Telemetry_RequestFlashParamSnapshot())
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_StartMotor(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    return (MC_Start_Motor() == MC_SUCCESS) ? CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_STATE;
}

static CanCmdStatus_t Can_StopMotor(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;
    (void)MC_Stop_Motor();
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetSpeedKp(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    float value;
    (void)len;
    (void)extra;

    value = Can_ReadFloatLE(data);
    if ((!isfinite(value)) || (value < 0.0f) || (value > CAN_CMD_GAIN_MAX))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    MC_Set_Speed_Kp(value);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetSpeedKi(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    float value;
    (void)len;
    (void)extra;

    value = Can_ReadFloatLE(data);
    if ((!isfinite(value)) || (value < 0.0f) || (value > CAN_CMD_GAIN_MAX))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    MC_Set_Speed_Ki(value);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetRefSpeed(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    float value;
    (void)len;
    (void)extra;

    value = Can_ReadFloatLE(data);
    if ((!isfinite(value)) || (fabsf(value) > CAN_CMD_SPEED_ABS_LIMIT_RPM))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    MC_Set_Speed_Reference(value);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetModeSpeed(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;
    MC_Set_Control_Mode(CTRL_MODE_SPEED);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetModePosition(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;
    MC_Set_Control_Mode(CTRL_MODE_POSITION);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetModeVf(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;
    MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetPolePairs(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    uint8_t polePairs;
    (void)len;
    (void)extra;

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        return CAN_CMD_STATUS_BUSY;
    }

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    polePairs = data[0];
    if (polePairs == 0U)
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    return (MC_Set_Pole_Pairs(polePairs) == MC_SUCCESS) ? CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_ARG;
}

static CanCmdStatus_t Can_CalibStart(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    MC_RetStatus_t ret = MC_FAILED;
    uint8_t step;
    (void)len;
    (void)extra;

    step = data[0];
    switch ((CanCalibCmd_t)step)
    {
        case CAN_CALIB_CHAIN_ALL:
            ret = MC_Calib_StartChain();
            break;

        case CAN_CALIB_PARAM_ALL:
            ret = MC_Calib_StartParam(PARAM_ID_STEP_ALL);
            break;

        default:
            return CAN_CMD_STATUS_BAD_ARG;
    }

    if (ret == MC_SUCCESS)
    {
        return CAN_CMD_STATUS_OK;
    }

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        return CAN_CMD_STATUS_BUSY;
    }

    return CAN_CMD_STATUS_BAD_STATE;
}

static CanCmdStatus_t Can_CalibStop(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    (void)extra;

    if (MC_Calib_StopParam() != MC_SUCCESS)
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_GetNodeId(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    (void)data;
    (void)len;
    *extra = ParamId_GetCanNodeId();
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetNodeId(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    uint8_t newNodeId;

    if (len != 1U)
    {
        return CAN_CMD_STATUS_BAD_LEN;
    }

    newNodeId = data[0];
    if (!ParamId_SaveCanNodeIdToFlash(newNodeId))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    if (!Can_UpdateRxFilterForCurrentNode())
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    s_canopen_rpdo1_cobid = CANOPEN_COBID_RPDO1_BASE + newNodeId;
    s_canopen_tpdo1_cobid = CANOPEN_COBID_TPDO1_BASE + newNodeId;
    s_canopen_rpdo2_cobid = CANOPEN_COBID_RPDO2_BASE + newNodeId;
    s_canopen_tpdo2_cobid = CANOPEN_COBID_TPDO2_BASE + newNodeId;

    *extra = newNodeId;
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_Cia402Controlword(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    uint16_t controlword;
    (void)extra;

    if (len != 2U)
    {
        return CAN_CMD_STATUS_BAD_LEN;
    }

    controlword = Can_ReadU16LE(data);
    return (MC_Apply_Cia402_Controlword(controlword) == MC_SUCCESS) ? CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_STATE;
}

static CanCmdStatus_t Can_Cia402Mode(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    int8_t mode;
    (void)extra;

    if (len != 1U)
    {
        return CAN_CMD_STATUS_BAD_LEN;
    }

    mode = (int8_t)data[0];
    return MC_Cia402_WriteObject(0x6060U, 0U, (const uint8_t *)&mode, 1U) ?
           CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_ARG;
}

static CanCmdStatus_t Can_Cia402TargetSpeed(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    int32_t targetRpm;
    uint8_t value[4];
    (void)extra;

    if (len != 4U)
    {
        return CAN_CMD_STATUS_BAD_LEN;
    }

    targetRpm = Can_ReadS32LE(data);
    value[0] = (uint8_t)targetRpm;
    value[1] = (uint8_t)(targetRpm >> 8);
    value[2] = (uint8_t)(targetRpm >> 16);
    value[3] = (uint8_t)(targetRpm >> 24);
    return MC_Cia402_WriteObject(0x60FFU, 0U, value, sizeof(value)) ?
           CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_ARG;
}

static CanCmdStatus_t Can_Cia402TargetTorque(const uint8_t *data, uint8_t len, uint8_t *extra)
{
    int16_t targetIq_mA;
    uint8_t value[2];
    (void)extra;

    if (len != 2U)
    {
        return CAN_CMD_STATUS_BAD_LEN;
    }

    targetIq_mA = Can_ReadS16LE(data);
    value[0] = (uint8_t)targetIq_mA;
    value[1] = (uint8_t)(targetIq_mA >> 8);
    return MC_Cia402_WriteObject(0x6071U, 0U, value, sizeof(value)) ?
           CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_ARG;
}

static const CanCmdTable_t s_can_cmd_table[] =
{
    {CAN_FC_STOP_MOTOR, 0U, Can_StopMotor},
    {CAN_FC_START_MOTOR, 0U, Can_StartMotor},
    {CAN_FC_SET_MODE_SPEED, 0U, Can_SetModeSpeed},
    {CAN_FC_SET_MODE_POS, 0U, Can_SetModePosition},
    {CAN_FC_SET_MODE_VF, 0U, Can_SetModeVf},
    {CAN_FC_SET_REF_SPEED, 4U, Can_SetRefSpeed},
    {CAN_FC_SET_SPEED_KP, 4U, Can_SetSpeedKp},
    {CAN_FC_SET_SPEED_KI, 4U, Can_SetSpeedKi},
    {CAN_FC_SET_POLE_PAIRS, 1U, Can_SetPolePairs},
    {CAN_FC_CALIB_START, 1U, Can_CalibStart},
    {CAN_FC_CALIB_STOP, 0U, Can_CalibStop},
    {CAN_FC_FLASH_READ_PARAM, 0U, Can_FlashReadParam},
    {CAN_FC_FLASH_CLEAR, 0U, Can_FlashClear},
    {CAN_FC_GET_ID, 0U, Can_GetNodeId},
    {CAN_FC_SET_ID, 1U, Can_SetNodeId},
#if APP_USE_CIA402_CAN
    {CAN_FC_CIA402_CONTROLWORD, 2U, Can_Cia402Controlword},
    {CAN_FC_CIA402_MODE, 1U, Can_Cia402Mode},
    {CAN_FC_CIA402_TARGET_SP, 4U, Can_Cia402TargetSpeed},
    {CAN_FC_CIA402_TARGET_TQ, 2U, Can_Cia402TargetTorque}
#endif
};

static CanCmdStatus_t Can_DispatchBySid(uint16_t sid, const uint8_t *data, uint8_t len, uint8_t *extraOut)
{
    uint32_t i;
    uint8_t funcCode = CAN_GET_FUNC(sid);
    uint8_t extra = 0U;

    if (extraOut != NULL)
    {
        *extraOut = 0U;
    }

    // 遍历CAN命令表，查找匹配的SID
    for (i = 0U; i < (sizeof(s_can_cmd_table) / sizeof(s_can_cmd_table[0])); i++)
    {
        if (s_can_cmd_table[i].funcCode == funcCode)
        {
            // 校验数据长度是否匹配
            if (len != s_can_cmd_table[i].expected_len)
            {
                s_mcp2518_status.rx_invalid_count++;
                return CAN_CMD_STATUS_BAD_LEN;
            }

            // 找到匹配项，调用对应的处理函数
            {
                CanCmdStatus_t status = s_can_cmd_table[i].handler(data, len, &extra);
                if (extraOut != NULL)
                {
                    *extraOut = extra;
                }
                return status;
            }
        }
    }

    // 未找到匹配的SID
    s_mcp2518_status.rx_invalid_count++;
    return CAN_CMD_STATUS_UNKNOWN;
}

HAL_StatusTypeDef DRV_SPI_TransferData(uint8_t spiDeviceIndex, uint8_t *SpiTxData,
	uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	HAL_StatusTypeDef status;
    (void)spiDeviceIndex;
	HAL_GPIO_WritePin(COMM_CS_N_GPIO_Port, COMM_CS_N_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(&hspi2, SpiTxData, SpiRxData, spiTransferSize, CAN_SPI_TRANSFER_TIMEOUT_MS);
	HAL_GPIO_WritePin(COMM_CS_N_GPIO_Port, COMM_CS_N_Pin, GPIO_PIN_SET);
	return status;
}

void CANFD_INIT(void)
{
	CAN_TX_FIFO_CONFIG txConfig;
	CAN_RX_FIFO_CONFIG rxConfig;
	CAN_CONFIG config;

    memset(&s_mcp2518_status, 0, sizeof(s_mcp2518_status));
    s_param_state_prev = MC_Calib_GetParamState();

	DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);
	DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);
	DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);
	DRV_CANFDSPI_ConfigureObjectReset(&config);

	config.IsoCrcEnable = 1;
	config.StoreInTEF = 0;
	config.BitRateSwitchDisable = 0;
	DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

	DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
	txConfig.FifoSize = 7;
	txConfig.PayLoadSize = APP_CAN_TX_FIFO_PAYLOAD_SIZE;
	txConfig.TxPriority = 0;
	DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &txConfig);

	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
	rxConfig.FifoSize = 15;
	rxConfig.PayLoadSize = APP_CAN_RX_FIFO_PAYLOAD_SIZE;
	rxConfig.RxTimeStampEnable = 0;
	DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &rxConfig);

	(void)Can_ConfigureRxFilter(ParamId_GetCanNodeId());
#if APP_USE_LEGACY_CAN_PROTOCOL
	(void)Can_ConfigureBroadcastGetIdFilter();
#endif
	DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, APP_CAN_BITTIME_SETUP, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_RX_EVENT);
	(void)DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
    s_canopen_nmt_state = CANOPEN_NMT_PREOP;
    MC_Cia402_ResetState();
    memset(&s_canopen_sdo_upload, 0, sizeof(s_canopen_sdo_upload));
    memset(&s_canopen_sdo_download, 0, sizeof(s_canopen_sdo_download));
    s_canopen_heartbeat_elapsed_ms = 0U;
    s_canopen_tpdo_elapsed_ms = 0U;
    s_canopen_tpdo2_elapsed_ms = 0U;
    s_canopen_last_error = AXIS_ERROR_NONE;
    s_canopen_rpdo1_cobid = CANOPEN_COBID_RPDO1_BASE +
                            ParamId_GetCanNodeId();
    s_canopen_tpdo1_cobid = CANOPEN_COBID_TPDO1_BASE +
                            ParamId_GetCanNodeId();
    s_canopen_rpdo2_cobid = CANOPEN_COBID_RPDO2_BASE +
                            ParamId_GetCanNodeId();
    s_canopen_tpdo2_cobid = CANOPEN_COBID_TPDO2_BASE +
                            ParamId_GetCanNodeId();
    s_canopen_rpdo1_map_count = 2U;
    s_canopen_tpdo1_map_count = 4U;
    s_canopen_rpdo2_map_count = 2U;
    s_canopen_tpdo2_map_count = 2U;
    s_canopen_rpdo1_mapping[0] = 0x60400010UL;
    s_canopen_rpdo1_mapping[1] = 0x60FF0020UL;
    s_canopen_tpdo1_mapping[0] = 0x60410010UL;
    s_canopen_tpdo1_mapping[1] = 0x606C0020UL;
    s_canopen_tpdo1_mapping[2] = 0x60610008UL;
    s_canopen_tpdo1_mapping[3] = 0x10010008UL;
    s_canopen_rpdo2_mapping[0] = 0x60600008UL;
    s_canopen_rpdo2_mapping[1] = 0x60710010UL;
    s_canopen_tpdo2_mapping[0] = 0x60640020UL;
    s_canopen_tpdo2_mapping[1] = 0x607A0020UL;
    s_canopen_rpdo1_transmission = 0xFFU;
    s_canopen_rpdo2_transmission = 0xFFU;
    s_canopen_tpdo1_transmission = 0xFFU;
    s_canopen_tpdo2_transmission = 0xFFU;
    s_canopen_tpdo1_enabled = 1U;
    s_canopen_tpdo2_enabled = 1U;
    s_canopen_tpdo1_inhibit_ms = 0U;
    s_canopen_tpdo2_inhibit_ms = 0U;
    s_canopen_tpdo1_event_ms = CANOPEN_TPDO_PERIOD_MS;
    s_canopen_tpdo2_event_ms = CANOPEN_TPDO_PERIOD_MS;
    CanOpen_SendBootup();
}

void MCP2518FD_TransmitMessageQueue(CANFDSPI_MODULE_ID index, uint16_t id, uint8_t *data, CAN_DLC len)
{
	CAN_TX_FIFO_EVENT txFlags;
	CAN_TX_MSGOBJ txObj;
    uint8_t n;

    if (!CanOpen_IsAllowedTxCobId((uint16_t)(id & 0x7FFU)))
    {
        s_mcp2518_status.tx_drop_count++;
        return;
    }

#ifdef APP_USE_TX_INT
    if (!APP_TX_INT())
    {
        s_mcp2518_status.tx_timeout_count++;
        s_mcp2518_status.tx_drop_count++;
        return;
    }
#else
    DRV_CANFDSPI_TransmitChannelEventGet(index, CAN_FIFO_CH2, &txFlags);
    if ((txFlags & CAN_TX_FIFO_NOT_FULL_EVENT) == 0U)
    {
        s_mcp2518_status.tx_timeout_count++;
        s_mcp2518_status.tx_drop_count++;
        return;
    }
#endif

    memset(&txObj, 0, sizeof(txObj));
    n = DRV_CANFDSPI_DlcToDataBytes(len);
    txObj.bF.id.SID = id & 0x7FFU;
    txObj.bF.ctrl.DLC = len;
    txObj.bF.ctrl.IDE = 0;
    txObj.bF.ctrl.RTR = 0;
    txObj.bF.ctrl.BRS = APP_CAN_FRAME_BRS;
    txObj.bF.ctrl.FDF = APP_CAN_FRAME_FDF;

    if (DRV_CANFDSPI_TransmitChannelLoad(index, CAN_FIFO_CH2, &txObj,
                                         data, n, true) != 0)
    {
        s_mcp2518_status.tx_timeout_count++;
        s_mcp2518_status.tx_drop_count++;
        return;
    }
    s_mcp2518_status.tx_frame_count++;
}

void MCP2518FD_ReceiveMessage(CANFDSPI_MODULE_ID index, uint8_t nBytes)
{
	CAN_RX_MSGOBJ rxObj;
	CAN_RX_FIFO_EVENT rxFlags;
	uint8_t rxdata[APP_CAN_RX_FETCH_BYTES];
	uint8_t dlcDataBytes;
#if APP_USE_LEGACY_CAN_PROTOCOL
    uint8_t cmdExtra;
    uint8_t responseNodeId;
    CanCmdStatus_t cmdStatus;
#endif

	DRV_CANFDSPI_ReceiveChannelEventGet(index, CAN_FIFO_CH1, &rxFlags);

	if ((rxFlags & CAN_RX_FIFO_OVERFLOW_EVENT) != 0U)
	{
		s_mcp2518_status.rx_overflow_count++;
	}

	while ((rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) != 0U)
	{
		if (DRV_CANFDSPI_ReceiveMessageGet(index, CAN_FIFO_CH1, &rxObj,
										   rxdata, nBytes) < 0)
		{
			s_mcp2518_status.rx_invalid_count++;
			break;
		}
		dlcDataBytes = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
		if (dlcDataBytes > APP_CAN_RX_FETCH_BYTES)
		{
			dlcDataBytes = APP_CAN_RX_FETCH_BYTES;
		}

		s_mcp2518_status.rx_frame_count++;
		s_mcp2518_status.last_rx_sid = rxObj.bF.id.SID;
		s_mcp2518_status.last_rx_len = dlcDataBytes;
        if (CanOpen_HandleStandardFrame(rxObj.bF.id.SID, rxdata, dlcDataBytes))
        {
            s_mcp2518_status.last_rx_status = (uint8_t)CAN_CMD_STATUS_OK;
            DRV_CANFDSPI_ReceiveChannelEventGet(index, CAN_FIFO_CH1, &rxFlags);
            continue;
        }
#if APP_USE_LEGACY_CAN_PROTOCOL
        cmdStatus = Can_DispatchBySid(rxObj.bF.id.SID, rxdata, dlcDataBytes, &cmdExtra);
        s_mcp2518_status.last_rx_status = (uint8_t)cmdStatus;
        if (cmdStatus != CAN_CMD_STATUS_OK)
        {
            s_mcp2518_status.rx_reject_count++;
        }
        responseNodeId = CAN_GET_NODE(rxObj.bF.id.SID);
        if ((CAN_GET_FUNC(rxObj.bF.id.SID) == CAN_FC_GET_ID) && (responseNodeId == 0U))
        {
            responseNodeId = ParamId_GetCanNodeId();
        }
        Can_SendCmdStatus(rxObj.bF.id.SID, cmdStatus, cmdExtra, responseNodeId);
#else
        /*
         * In standard-only mode, an unrecognized frame is intentionally
         * ignored instead of generating a legacy command-status response.
         */
        s_mcp2518_status.last_rx_status = (uint8_t)CAN_CMD_STATUS_UNKNOWN;
        s_mcp2518_status.rx_invalid_count++;
#endif
		DRV_CANFDSPI_ReceiveChannelEventGet(index, CAN_FIFO_CH1, &rxFlags);
	}
}

void MCP2518FD_ProcessRxIrq(void)
{
	s_mcp2518_status.rx_irq_count++;
	MCP2518FD_ReceiveMessage(DRV_CANFDSPI_INDEX_0, APP_CAN_RX_FETCH_BYTES);
}

void MCP2518FD_Service1ms(void)
{
    ParamIdState_t stateNow;
    AxisError_t errorNow = g_axis.error;

    stateNow = MC_Calib_GetParamState();
    if (stateNow != s_param_state_prev)
    {
        s_param_state_prev = stateNow;
        CAN_Telemetry_RequestRuntimeParamSnapshot();
    }

    if (errorNow != s_canopen_last_error)
    {
        s_canopen_last_error = errorNow;
        if (errorNow != AXIS_ERROR_NONE)
        {
            CanOpen_SendEmcy(errorNow);
        }
    }

    if (s_canopen_nmt_state != CANOPEN_NMT_INITIALIZING)
    {
        s_canopen_heartbeat_elapsed_ms++;
        if ((s_canopen_heartbeat_ms != 0U) &&
            (s_canopen_heartbeat_elapsed_ms >= s_canopen_heartbeat_ms))
        {
            s_canopen_heartbeat_elapsed_ms = 0U;
            CanOpen_SendHeartbeat();
        }
    }

    if (s_canopen_nmt_state == CANOPEN_NMT_OPERATIONAL)
    {
        s_canopen_tpdo_elapsed_ms++;
        {
            uint16_t period = (s_canopen_tpdo1_event_ms == 0U) ?
                              CANOPEN_TPDO_PERIOD_MS :
                              s_canopen_tpdo1_event_ms;
            if (s_canopen_tpdo1_inhibit_ms > period)
            {
                period = s_canopen_tpdo1_inhibit_ms;
            }
            if (((s_canopen_tpdo1_transmission == 254U) ||
                 (s_canopen_tpdo1_transmission == 255U)) &&
                (s_canopen_tpdo_elapsed_ms >= period))
            {
                s_canopen_tpdo_elapsed_ms = 0U;
                CanOpen_SendTpdo();
            }
        }
        s_canopen_tpdo2_elapsed_ms++;
        {
            uint16_t period = (s_canopen_tpdo2_event_ms == 0U) ?
                              CANOPEN_TPDO_PERIOD_MS :
                              s_canopen_tpdo2_event_ms;
            if (s_canopen_tpdo2_inhibit_ms > period)
            {
                period = s_canopen_tpdo2_inhibit_ms;
            }
            if (((s_canopen_tpdo2_transmission == 254U) ||
                 (s_canopen_tpdo2_transmission == 255U)) &&
                (s_canopen_tpdo2_elapsed_ms >= period))
            {
                s_canopen_tpdo2_elapsed_ms = 0U;
                CanOpen_SendTpdo2();
            }
        }
    }
}

MCP2518FD_Status_t MCP2518FD_GetStatus(void)
{
	return s_mcp2518_status;
}
