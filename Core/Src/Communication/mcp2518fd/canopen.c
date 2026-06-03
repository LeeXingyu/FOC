#include "canopen.h"
#include "can_telemetry.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "param_identify.h"

#include <math.h>

static MCP2518FD_Status_t s_mcp2518_status = {0};
static ParamIdState_t s_param_state_prev = PARAM_ID_STATE_IDLE;

#define CAN_CMD_START_MOTOR          0x101U
#define CAN_CMD_STOP_MOTOR           0x102U
#define CAN_CMD_SET_SPEED_KP         0x103U
#define CAN_CMD_SET_SPEED_KI         0x104U
#define CAN_CMD_SET_REF_SPEED        0x105U
#define CAN_CMD_SET_MODE_SPEED       0x106U
#define CAN_CMD_SET_MODE_POS         0x107U
#define CAN_CMD_SET_MODE_VF          0x108U
#define CAN_CMD_SET_POLE_PAIRS       0x109U
#define CAN_CMD_CALIB_START          0x10AU
#define CAN_CMD_CALIB_STOP           0x10BU
#define CAN_CMD_FLASH_CLEAR          0x10CU
#define CAN_CMD_FLASH_READ_PARAM     0x10DU

#define CAN_CMD_SPEED_ABS_LIMIT_RPM  30000.0f
#define CAN_CMD_GAIN_MAX             1000.0f

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

typedef CanCmdStatus_t (*CanCmdHandler)(const uint8_t *data, uint8_t len);

typedef struct
{
    uint16_t sid;
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

static void Can_SendCmdStatus(uint16_t cmdSid, CanCmdStatus_t status, uint8_t extra)
{
    CAN_Telemetry_QueueCmdStatus(cmdSid, (uint8_t)status, extra, s_mcp2518_status.last_rx_len);
}

static CanCmdStatus_t Can_FlashClear(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

    if (MC_Calib_GetParamState() == PARAM_ID_STATE_PREPARE ||
        MC_Calib_GetParamState() == PARAM_ID_STATE_LOCK_CHECK ||
        MC_Calib_GetParamState() == PARAM_ID_STATE_RUN)
    {
        return CAN_CMD_STATUS_BUSY;
    }

    return ParamId_ClearFlash() ? CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_STATE;
}

static CanCmdStatus_t Can_FlashReadParam(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

    if (!CAN_Telemetry_RequestFlashParamSnapshot())
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_StartMotor(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    return (MC_Start_Motor() == MC_SUCCESS) ? CAN_CMD_STATUS_OK : CAN_CMD_STATUS_BAD_STATE;
}

static CanCmdStatus_t Can_StopMotor(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    (void)MC_Stop_Motor();
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetSpeedKp(const uint8_t *data, uint8_t len)
{
    float value;
    (void)len;

    value = Can_ReadFloatLE(data);
    if ((!isfinite(value)) || (value < 0.0f) || (value > CAN_CMD_GAIN_MAX))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    MC_Set_Speed_Kp(value);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetSpeedKi(const uint8_t *data, uint8_t len)
{
    float value;
    (void)len;

    value = Can_ReadFloatLE(data);
    if ((!isfinite(value)) || (value < 0.0f) || (value > CAN_CMD_GAIN_MAX))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    MC_Set_Speed_Ki(value);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetRefSpeed(const uint8_t *data, uint8_t len)
{
    float value;
    (void)len;

    value = Can_ReadFloatLE(data);
    if ((!isfinite(value)) || (fabsf(value) > CAN_CMD_SPEED_ABS_LIMIT_RPM))
    {
        return CAN_CMD_STATUS_BAD_ARG;
    }

    MC_Set_Speed_Reference(value);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetModeSpeed(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    MC_Set_Control_Mode(CTRL_MODE_SPEED);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetModePosition(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    MC_Set_Control_Mode(CTRL_MODE_POSITION);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetModeVf(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;
    MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
    return CAN_CMD_STATUS_OK;
}

static CanCmdStatus_t Can_SetPolePairs(const uint8_t *data, uint8_t len)
{
    ParamIdState_t paramState = MC_Calib_GetParamState();
    uint8_t polePairs;
    (void)len;

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

static CanCmdStatus_t Can_CalibStart(const uint8_t *data, uint8_t len)
{
    MC_RetStatus_t ret = MC_FAILED;
    uint8_t step;
    (void)len;

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

static CanCmdStatus_t Can_CalibStop(const uint8_t *data, uint8_t len)
{
    (void)data;
    (void)len;

    if (MC_Calib_StopParam() != MC_SUCCESS)
    {
        return CAN_CMD_STATUS_BAD_STATE;
    }

    return CAN_CMD_STATUS_OK;
}

static const CanCmdTable_t s_can_cmd_table[] =
{
    {CAN_CMD_START_MOTOR, 0U, Can_StartMotor},
    {CAN_CMD_STOP_MOTOR, 0U, Can_StopMotor},
    {CAN_CMD_SET_SPEED_KP, 4U, Can_SetSpeedKp},
    {CAN_CMD_SET_SPEED_KI, 4U, Can_SetSpeedKi},
    {CAN_CMD_SET_REF_SPEED, 4U, Can_SetRefSpeed},
    {CAN_CMD_SET_MODE_SPEED, 0U, Can_SetModeSpeed},
    {CAN_CMD_SET_MODE_POS, 0U, Can_SetModePosition},
    {CAN_CMD_SET_MODE_VF, 0U, Can_SetModeVf},
    {CAN_CMD_SET_POLE_PAIRS, 1U, Can_SetPolePairs},
    {CAN_CMD_CALIB_START, 1U, Can_CalibStart},
    {CAN_CMD_CALIB_STOP, 0U, Can_CalibStop},
    {CAN_CMD_FLASH_CLEAR, 0U, Can_FlashClear},
    {CAN_CMD_FLASH_READ_PARAM, 0U, Can_FlashReadParam}
};

static CanCmdStatus_t Can_DispatchBySid(uint16_t sid, const uint8_t *data, uint8_t len)
{
    uint32_t i;

    for (i = 0U; i < (sizeof(s_can_cmd_table) / sizeof(s_can_cmd_table[0])); i++)
    {
        if (s_can_cmd_table[i].sid == sid)
        {
            if (len != s_can_cmd_table[i].expected_len)
            {
                s_mcp2518_status.rx_invalid_count++;
                return CAN_CMD_STATUS_BAD_LEN;
            }

            return s_can_cmd_table[i].handler(data, len);
        }
    }

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
	REG_CiFLTOBJ fObj;
	REG_CiMASK mObj;
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
	txConfig.PayLoadSize = CAN_PLSIZE_16;
	txConfig.TxPriority = 0;
	DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH2, &txConfig);

	DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
	rxConfig.FifoSize = 15;
	rxConfig.PayLoadSize = CAN_PLSIZE_16;
	rxConfig.RxTimeStampEnable = 0;
	DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, &rxConfig);

	fObj.word = 0;
	fObj.bF.SID = 0x000;
	fObj.bF.SID11 = 0;
	fObj.bF.EXIDE = 0;
	fObj.bF.EID = 0;
	DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);

	mObj.word = 0;
	mObj.bF.MSID = 0x000;
	mObj.bF.MSID11 = 0;
	mObj.bF.MIDE = 1;
	mObj.bF.MEID = 0;
	DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);

	DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, CAN_FIFO_CH1, true);
	DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_1000K_4M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
	DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
	DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, CAN_FIFO_CH1, CAN_RX_FIFO_NOT_EMPTY_EVENT);
	DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_RX_EVENT);
	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
}

void MCP2518FD_TransmitMessageQueue(CANFDSPI_MODULE_ID index, uint16_t id, uint8_t *data, CAN_DLC len)
{
	CAN_TX_FIFO_EVENT txFlags;
	CAN_TX_MSGOBJ txObj;
    uint8_t n;

    do
    {
#ifdef APP_USE_TX_INT
        HAL_Delay(50);
#else
        DRV_CANFDSPI_TransmitChannelEventGet(index, CAN_FIFO_CH2, &txFlags);
#endif
    }
#ifdef APP_USE_TX_INT
    while (!APP_TX_INT());
#else
    while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));
#endif

    n = DRV_CANFDSPI_DlcToDataBytes(len);
    txObj.bF.id.SID = id;
    txObj.bF.ctrl.DLC = len;
    txObj.bF.ctrl.IDE = 0;
    txObj.bF.ctrl.RTR = 0;
    txObj.bF.ctrl.BRS = 0;
    txObj.bF.ctrl.FDF = 0;

    DRV_CANFDSPI_TransmitChannelLoad(index, CAN_FIFO_CH2, &txObj, data, n, true);
    s_mcp2518_status.tx_frame_count++;
}

void MCP2518FD_ReceiveMessage(CANFDSPI_MODULE_ID index, uint8_t nBytes)
{
	CAN_RX_MSGOBJ rxObj;
	CAN_RX_FIFO_EVENT rxFlags;
	uint8_t rxdata[8];
	uint8_t dlcDataBytes;
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
		if (dlcDataBytes > 8U)
		{
			dlcDataBytes = 8U;
		}

		s_mcp2518_status.rx_frame_count++;
		s_mcp2518_status.last_rx_sid = rxObj.bF.id.SID;
		s_mcp2518_status.last_rx_len = dlcDataBytes;
        cmdStatus = Can_DispatchBySid(rxObj.bF.id.SID, rxdata, dlcDataBytes);
        s_mcp2518_status.last_rx_status = (uint8_t)cmdStatus;
        if (cmdStatus != CAN_CMD_STATUS_OK)
        {
            s_mcp2518_status.rx_reject_count++;
        }
        Can_SendCmdStatus(rxObj.bF.id.SID, cmdStatus, 0U);
		DRV_CANFDSPI_ReceiveChannelEventGet(index, CAN_FIFO_CH1, &rxFlags);
	}
}

void MCP2518FD_ProcessRxIrq(void)
{
	s_mcp2518_status.rx_irq_count++;
	MCP2518FD_ReceiveMessage(DRV_CANFDSPI_INDEX_0, 8U);
}

void MCP2518FD_Service1ms(void)
{
    ParamIdState_t stateNow;

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
