#include "canopen.h"
#include "can_telemetry.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "param_identify.h"

#include <math.h>

static MCP2518FD_Status_t s_mcp2518_status = {0};
static ParamIdState_t s_param_state_prev = PARAM_ID_STATE_IDLE;

#define CAN_CMD_SPEED_ABS_LIMIT_RPM  1000.0f
#define CAN_CMD_GAIN_MAX             1000.0f
#define CAN_TX_WAIT_TIMEOUT_MS       5U

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
    fObj.bF.SID = nodeId & CAN_NODE_ID_MASK;
    fObj.bF.SID11 = 0U;
    fObj.bF.EXIDE = 0U;
    fObj.bF.EID = 0U;
    if (DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF) != 0)
    {
        return false;
    }

    mObj.word = 0U;
    mObj.bF.MSID = CAN_NODE_ID_MASK;
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
    ParamIdState_t paramState = MC_Calib_GetParamState();
    uint8_t polePairs;
    (void)len;
    (void)extra;

    if (paramState == PARAM_ID_STATE_PREPARE ||
        paramState == PARAM_ID_STATE_LOCK_CHECK ||
        paramState == PARAM_ID_STATE_RUN)
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

    if (MC_Calib_GetParamState() == PARAM_ID_STATE_PREPARE ||
        MC_Calib_GetParamState() == PARAM_ID_STATE_LOCK_CHECK ||
        MC_Calib_GetParamState() == PARAM_ID_STATE_RUN)
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

    *extra = newNodeId;
    return CAN_CMD_STATUS_OK;
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
    {CAN_FC_SET_ID, 1U, Can_SetNodeId}
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
	status = HAL_SPI_TransmitReceive(&hspi2, SpiTxData, SpiRxData, spiTransferSize, 1000);
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
	(void)Can_ConfigureBroadcastGetIdFilter();
	DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, APP_CAN_BITTIME_SETUP, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_RX_EVENT);
	(void)DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
}

void MCP2518FD_TransmitMessageQueue(CANFDSPI_MODULE_ID index, uint16_t id, uint8_t *data, CAN_DLC len)
{
	CAN_TX_FIFO_EVENT txFlags;
	CAN_TX_MSGOBJ txObj;
    uint8_t n;
    
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

    n = DRV_CANFDSPI_DlcToDataBytes(len);
    txObj.bF.id.SID = id;
    txObj.bF.ctrl.DLC = len;
    txObj.bF.ctrl.IDE = 0;
    txObj.bF.ctrl.RTR = 0;
    txObj.bF.ctrl.BRS = APP_CAN_FRAME_BRS;
    txObj.bF.ctrl.FDF = APP_CAN_FRAME_FDF;

    DRV_CANFDSPI_TransmitChannelLoad(index, CAN_FIFO_CH2, &txObj, data, n, true);
    s_mcp2518_status.tx_frame_count++;
}

void MCP2518FD_ReceiveMessage(CANFDSPI_MODULE_ID index, uint8_t nBytes)
{
	CAN_RX_MSGOBJ rxObj;
	CAN_RX_FIFO_EVENT rxFlags;
	uint8_t rxdata[APP_CAN_RX_FETCH_BYTES];
	uint8_t dlcDataBytes;
    uint8_t cmdExtra;
    uint8_t responseNodeId;
    CanCmdStatus_t cmdStatus;

	DRV_CANFDSPI_ReceiveChannelEventGet(index, CAN_FIFO_CH1, &rxFlags);

	if ((rxFlags & CAN_RX_FIFO_OVERFLOW_EVENT) != 0U)
	{
		s_mcp2518_status.rx_overflow_count++;
	}

	while ((rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) != 0U)
	{
		DRV_CANFDSPI_ReceiveMessageGet(index, CAN_FIFO_CH1, &rxObj, rxdata, nBytes);
		dlcDataBytes = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC)rxObj.bF.ctrl.DLC);
		if (dlcDataBytes > APP_CAN_RX_FETCH_BYTES)
		{
			dlcDataBytes = APP_CAN_RX_FETCH_BYTES;
		}

		s_mcp2518_status.rx_frame_count++;
		s_mcp2518_status.last_rx_sid = rxObj.bF.id.SID;
		s_mcp2518_status.last_rx_len = dlcDataBytes;
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

    /* Poll RX FIFO as a safety net in case the interrupt edge is missed. */
    MCP2518FD_ReceiveMessage(DRV_CANFDSPI_INDEX_0, APP_CAN_RX_FETCH_BYTES);

    stateNow = MC_Calib_GetParamState();
    if (stateNow != s_param_state_prev)
    {
        s_param_state_prev = stateNow;
        CAN_Telemetry_RequestRuntimeParamSnapshot();
    }
}

MCP2518FD_Status_t MCP2518FD_GetStatus(void)
{
	return s_mcp2518_status;
}
