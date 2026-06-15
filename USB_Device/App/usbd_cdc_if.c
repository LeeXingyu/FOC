/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          : USB CDC debug protocol bridge.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "usbd_cdc_if.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>

#include "main.h"
#include "cmsis_os.h"
#include "Communication/cdc_debug.h"
#include "MotorControl/Core/mc_interface.h"
#include "MotorControl/Tasks/mc_tasks.h"
#include "MotorControl/Control/param_identify.h"
#include "MotorControl/Fbdk/encoder.h"
#include "MotorControl/Fbdk/speed_pos_fbdk.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

#define CDC_RX_LINE_MAX        128U
#define CDC_TX_LINE_MAX        192U
#define CDC_TELEM_PERIOD_MS    20U

typedef struct
{
    char line[CDC_RX_LINE_MAX];
    uint16_t len;
    volatile uint8_t ready;
} CDC_RxState_t;

typedef struct
{
    CDC_DebugTelemetry_t latest;
    uint32_t last_tick_ms;
    uint8_t valid;
} CDC_TelemetryCache_t;

static CDC_RxState_t s_cdc_rx = {0};
static CDC_TelemetryCache_t s_cdc_telem = {0};
static uint8_t s_cdc_tx_busy = 0U;

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static uint8_t CDC_SendString(const char *str);
static uint8_t CDC_SendFormat(const char *fmt, ...);
static void CDC_AppendFixed6(char *dst, size_t dst_len, float value);
static void CDC_HandleLine(const char *line);
static void CDC_SendTelemetryLine(void);
static bool CDC_ParseU8(const char *text, uint8_t *value);
static bool CDC_ParseFloat(const char *text, float *value);
static void CDC_TrimInPlace(char *s);
static uint8_t CDC_HexNibble(char c);
static bool CDC_ParseHexBytes(const char *text, uint8_t *out, uint8_t maxLen, uint8_t *outLen);

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS
};

void CDC_Debug_Init(void)
{
    s_cdc_rx.len = 0U;
    s_cdc_rx.ready = 0U;
    s_cdc_tx_busy = 0U;
    s_cdc_telem.last_tick_ms = 0U;
    s_cdc_telem.valid = 0U;
}

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    CDC_Debug_Init();
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    (void)cmd;
    (void)pbuf;
    (void)length;
    return USBD_OK;
}

static void CDC_AppendRxBytes(const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    for (i = 0U; i < len; i++)
    {
        char c = (char)buf[i];

        if ((c == '\r') || (c == '\n'))
        {
            if (s_cdc_rx.len > 0U)
            {
                s_cdc_rx.line[s_cdc_rx.len] = '\0';
                s_cdc_rx.ready = 1U;
            }
            continue;
        }

        if (s_cdc_rx.len < (CDC_RX_LINE_MAX - 1U))
        {
            s_cdc_rx.line[s_cdc_rx.len++] = c;
        }
        else
        {
            s_cdc_rx.len = 0U;
        }
    }
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    CDC_AppendRxBytes(Buf, *Len);
    return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    if ((hcdc == NULL) || (hcdc->TxState != 0U) || (s_cdc_tx_busy != 0U))
    {
        return USBD_BUSY;
    }

    s_cdc_tx_busy = 1U;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
    {
        s_cdc_tx_busy = 0U;
        return USBD_BUSY;
    }

    return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
    (void)pbuf;
    (void)Len;
    (void)epnum;
    s_cdc_tx_busy = 0U;
    return USBD_OK;
}

static uint8_t CDC_SendString(const char *str)
{
    if (str == NULL)
    {
        return USBD_FAIL;
    }

    return CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
}

static uint8_t CDC_SendFormat(const char *fmt, ...)
{
    char buf[CDC_TX_LINE_MAX];
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return USBD_FAIL;
    }

    if ((size_t)n >= sizeof(buf))
    {
        n = (int)sizeof(buf) - 1;
        buf[n] = '\0';
    }

    return CDC_Transmit_FS((uint8_t *)buf, (uint16_t)strlen(buf));
}

static void CDC_AppendFixed6(char *dst, size_t dst_len, float value)
{
    double abs_v;
    unsigned long whole;
    unsigned long frac;
    int n;
    const char *sign = "";

    if ((dst == NULL) || (dst_len == 0U))
    {
        return;
    }

    if (value < 0.0f)
    {
        sign = "-";
        abs_v = -(double)value;
    }
    else
    {
        abs_v = (double)value;
    }

    whole = (unsigned long)abs_v;
    frac = (unsigned long)(((abs_v - (double)whole) * 1000000.0) + 0.5);
    if (frac >= 1000000UL)
    {
        whole += 1UL;
        frac -= 1000000UL;
    }

    n = snprintf(dst, dst_len, "%s%lu.%06lu", sign, whole, frac);
    if ((n < 0) || ((size_t)n >= dst_len))
    {
        dst[dst_len - 1U] = '\0';
    }
}

static void CDC_TrimInPlace(char *s)
{
    char *start;
    char *end;

    if (s == NULL)
    {
        return;
    }

    start = s;
    while ((*start != '\0') && isspace((unsigned char)*start))
    {
        start++;
    }

    if (start != s)
    {
        (void)memmove(s, start, strlen(start) + 1U);
    }

    end = s + strlen(s);
    while ((end > s) && isspace((unsigned char)end[-1]))
    {
        end--;
    }
    *end = '\0';
}

static bool CDC_ParseU8(const char *text, uint8_t *value)
{
    char *end = NULL;
    long v;

    if ((text == NULL) || (value == NULL))
    {
        return false;
    }

    v = strtol(text, &end, 0);
    if ((end == text) || (*end != '\0') || (v < 0) || (v > 255))
    {
        return false;
    }

    *value = (uint8_t)v;
    return true;
}

static bool CDC_ParseFloat(const char *text, float *value)
{
    char *end = NULL;
    float v;

    if ((text == NULL) || (value == NULL))
    {
        return false;
    }

    v = strtof(text, &end);
    if ((end == text) || (*end != '\0'))
    {
        return false;
    }

    *value = v;
    return true;
}

static uint8_t CDC_HexNibble(char c)
{
    if ((c >= '0') && (c <= '9')) return (uint8_t)(c - '0');
    if ((c >= 'a') && (c <= 'f')) return (uint8_t)(10 + c - 'a');
    if ((c >= 'A') && (c <= 'F')) return (uint8_t)(10 + c - 'A');
    return 0xFFU;
}

static bool CDC_ParseHexBytes(const char *text, uint8_t *out, uint8_t maxLen, uint8_t *outLen)
{
    uint8_t count = 0U;

    if ((text == NULL) || (out == NULL) || (outLen == NULL))
    {
        return false;
    }

    while (*text != '\0')
    {
        while ((*text == ' ') || (*text == '\t') || (*text == ','))
        {
            text++;
        }

        if (*text == '\0')
        {
            break;
        }

        if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X')))
        {
            text += 2;
        }

        if ((text[0] == '\0') || (text[1] == '\0'))
        {
            return false;
        }

        if (count >= maxLen)
        {
            return false;
        }

        {
            uint8_t hi = CDC_HexNibble(text[0]);
            uint8_t lo = CDC_HexNibble(text[1]);
            if ((hi == 0xFFU) || (lo == 0xFFU))
            {
                return false;
            }
            out[count++] = (uint8_t)((hi << 4) | lo);
        }

        text += 2;
    }

    *outLen = count;
    return true;
}

static void CDC_QueueTelemetry(const CDC_DebugTelemetry_t *t)
{
    if (t == NULL)
    {
        return;
    }
    s_cdc_telem.latest = *t;
    s_cdc_telem.valid = 1U;
    s_cdc_telem.last_tick_ms = HAL_GetTick();
}

void CDC_Debug_SetTelemetry(const CDC_DebugTelemetry_t *telemetry)
{
    CDC_QueueTelemetry(telemetry);
}

bool CDC_Debug_Enabled(void)
{
    return true;
}

static void CDC_SendTelemetryLine(void)
{
    char pos[24];
    char poss[24];
    char mech[24];
    char app[24];
    char app_pu[24];
    char spd[24];
    char id[24];
    char iq[24];
    char iqr[24];
    char line[CDC_TX_LINE_MAX];

    if (s_cdc_telem.valid == 0U)
    {
        return;
    }

    CDC_AppendFixed6(pos, sizeof(pos), s_cdc_telem.latest.position_deg);
    CDC_AppendFixed6(poss, sizeof(poss), s_cdc_telem.latest.position_cont_deg);
    CDC_AppendFixed6(mech, sizeof(mech), s_cdc_telem.latest.position_mech_deg);
    CDC_AppendFixed6(app, sizeof(app), s_cdc_telem.latest.position_app_deg);
    CDC_AppendFixed6(app_pu, sizeof(app_pu), s_cdc_telem.latest.position_app_pu);
    CDC_AppendFixed6(spd, sizeof(spd), s_cdc_telem.latest.speed_rpm);
    CDC_AppendFixed6(id, sizeof(id), s_cdc_telem.latest.current_d_a);
    CDC_AppendFixed6(iq, sizeof(iq), s_cdc_telem.latest.current_q_a);
    CDC_AppendFixed6(iqr, sizeof(iqr), s_cdc_telem.latest.current_ref_q_a);

    (void)snprintf(line,
                   sizeof(line),
                   "TEL POS=%s POSS=%s MECH=%s APPPU=%s APP=%s SPD=%s ID=%s IQ=%s IQR=%s AST=%u ERR=%u MODE=%u PST=%u\r\n",
                   pos,
                   poss,
                   mech,
                   app_pu,
                   app,
                   spd,
                   id,
                   iq,
                   iqr,
                   s_cdc_telem.latest.axis_state,
                   s_cdc_telem.latest.axis_error,
                   s_cdc_telem.latest.control_mode,
                   s_cdc_telem.latest.param_state);
    (void)CDC_SendString(line);
}

static void CDC_HandleCanLikeCommand(uint8_t func, uint8_t node, const uint8_t *payload, uint8_t len)
{
    uint16_t sid = (uint16_t)(((uint16_t)func << 4) | (node & 0x0FU));

    switch (func)
    {
        case 0x01U:
            (void)MC_Stop_Motor();
            CDC_SendFormat("DBG CMD 0x%03X stop\r\n", sid);
            break;
        case 0x02U:
            if (MC_Start_Motor() == MC_SUCCESS)
            {
                CDC_SendFormat("DBG CMD 0x%03X start\r\n", sid);
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X start\r\n", sid);
            }
            break;
        case 0x03U:
            MC_Set_Control_Mode(CTRL_MODE_SPEED);
            CDC_SendFormat("DBG CMD 0x%03X speedmode\r\n", sid);
            break;
        case 0x04U:
            MC_Set_Control_Mode(CTRL_MODE_POSITION);
            CDC_SendFormat("DBG CMD 0x%03X posmode\r\n", sid);
            break;
        case 0x05U:
            MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
            CDC_SendFormat("DBG CMD 0x%03X vfmode\r\n", sid);
            break;
        case 0x06U:
            if (len == 4U)
            {
                float v;
                memcpy(&v, payload, 4U);
                MC_Set_Speed_Reference(v);
                CDC_SendFormat("DBG CMD 0x%03X speed=%.3f\r\n", sid, (double)v);
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X bad_len\r\n", sid);
            }
            break;
        case 0x07U:
            if (len == 4U)
            {
                float v;
                memcpy(&v, payload, 4U);
                MC_Set_Speed_Kp(v);
                CDC_SendFormat("DBG CMD 0x%03X kp=%.3f\r\n", sid, (double)v);
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X bad_len\r\n", sid);
            }
            break;
        case 0x08U:
            if (len == 4U)
            {
                float v;
                memcpy(&v, payload, 4U);
                MC_Set_Speed_Ki(v);
                CDC_SendFormat("DBG CMD 0x%03X ki=%.3f\r\n", sid, (double)v);
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X bad_len\r\n", sid);
            }
            break;
        case 0x09U:
            if (len == 1U)
            {
                if (MC_Set_Pole_Pairs(payload[0]) == MC_SUCCESS)
                {
                    CDC_SendFormat("DBG CMD 0x%03X pole=%u\r\n", sid, payload[0]);
                }
                else
                {
                    CDC_SendFormat("DBG ERR 0x%03X bad_arg\r\n", sid);
                }
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X bad_len\r\n", sid);
            }
            break;
        case 0x0AU:
            if (len == 1U)
            {
                MC_RetStatus_t ret = (payload[0] == 0U) ? MC_Calib_StartChain() : MC_Calib_StartParam(PARAM_ID_STEP_ALL);
                CDC_SendFormat((ret == MC_SUCCESS) ? "DBG CMD 0x%03X calib=%u\r\n" : "DBG ERR 0x%03X calib\r\n",
                               sid,
                               payload[0]);
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X bad_len\r\n", sid);
            }
            break;
        case 0x0BU:
            CDC_SendFormat((MC_Calib_StopParam() == MC_SUCCESS) ? "DBG CMD 0x%03X calibstop\r\n" : "DBG ERR 0x%03X calibstop\r\n", sid);
            break;
        case 0x0CU:
            CDC_SendFormat((ParamId_RestoreFromFlashToAxis() ? "DBG CMD 0x%03X flashread\r\n" : "DBG ERR 0x%03X flashread\r\n"), sid);
            break;
        case 0x0DU:
            CDC_SendFormat((ParamId_ClearFlash() ? "DBG CMD 0x%03X flashclear\r\n" : "DBG ERR 0x%03X flashclear\r\n"), sid);
            break;
        case 0x0EU:
            CDC_SendFormat("DBG CMD 0x%03X node=%u\r\n", sid, ParamId_GetCanNodeId());
            break;
        case 0x0FU:
            if (len == 1U)
            {
                CDC_SendFormat((ParamId_SaveCanNodeIdToFlash(payload[0]) ? "DBG CMD 0x%03X node=%u\r\n" : "DBG ERR 0x%03X node\r\n"), sid, payload[0]);
            }
            else
            {
                CDC_SendFormat("DBG ERR 0x%03X bad_len\r\n", sid);
            }
            break;
        default:
            CDC_SendFormat("DBG ERR 0x%03X unknown\r\n", sid);
            break;
    }
}

static void CDC_HandleLine(const char *line)
{
    char local[CDC_RX_LINE_MAX];
    char *cmd = NULL;
    char *arg1 = NULL;
    char *arg2 = NULL;
    char *arg3 = NULL;
    uint8_t bytes[8] = {0};
    uint8_t byteLen = 0U;
    uint8_t func = 0U;
    uint8_t node = 0U;

    if (line == NULL)
    {
        return;
    }

    (void)strncpy(local, line, sizeof(local) - 1U);
    local[sizeof(local) - 1U] = '\0';
    CDC_TrimInPlace(local);
    if (local[0] == '\0')
    {
        return;
    }

    cmd = strtok(local, " \t");
    if (cmd == NULL)
    {
        return;
    }

    arg1 = strtok(NULL, " \t");
    arg2 = strtok(NULL, " \t");
    arg3 = strtok(NULL, " \t");

    if (strcmp(cmd, "help") == 0)
    {
        CDC_SendString(
            "DBG CDC command set:\r\n"
            "  start | stop | speedmode | posmode | vfmode\r\n"
            "  speed <rpm>\r\n"
            "  kp <value>\r\n"
            "  ki <value>\r\n"
            "  pole <n>\r\n"
            "  nodeget | nodeset <0-15>\r\n"
            "  flashread | flashclear\r\n"
            "  calib <0|5> | calibstop\r\n"
            "  can <func_hex> <node> <hexbytes>\r\n");
        return;
    }

    if (strcmp(cmd, "start") == 0)
    {
        CDC_HandleCanLikeCommand(0x02U, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "stop") == 0)
    {
        CDC_HandleCanLikeCommand(0x01U, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "speedmode") == 0)
    {
        CDC_HandleCanLikeCommand(0x03U, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "posmode") == 0)
    {
        CDC_HandleCanLikeCommand(0x04U, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "vfmode") == 0)
    {
        CDC_HandleCanLikeCommand(0x05U, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "speed") == 0)
    {
        float v;
        if (CDC_ParseFloat(arg1, &v))
        {
            memcpy(bytes, &v, 4U);
            CDC_HandleCanLikeCommand(0x06U, ParamId_GetCanNodeId(), bytes, 4U);
        }
        else
        {
            CDC_SendString("DBG ERR bad speed\r\n");
        }
        return;
    }
    if (strcmp(cmd, "kp") == 0)
    {
        float v;
        if (CDC_ParseFloat(arg1, &v))
        {
            memcpy(bytes, &v, 4U);
            CDC_HandleCanLikeCommand(0x07U, ParamId_GetCanNodeId(), bytes, 4U);
        }
        else
        {
            CDC_SendString("DBG ERR bad kp\r\n");
        }
        return;
    }
    if (strcmp(cmd, "ki") == 0)
    {
        float v;
        if (CDC_ParseFloat(arg1, &v))
        {
            memcpy(bytes, &v, 4U);
            CDC_HandleCanLikeCommand(0x08U, ParamId_GetCanNodeId(), bytes, 4U);
        }
        else
        {
            CDC_SendString("DBG ERR bad ki\r\n");
        }
        return;
    }
    if (strcmp(cmd, "pole") == 0)
    {
        if (CDC_ParseU8(arg1, &bytes[0]))
        {
            CDC_HandleCanLikeCommand(0x09U, ParamId_GetCanNodeId(), bytes, 1U);
        }
        else
        {
            CDC_SendString("DBG ERR bad pole\r\n");
        }
        return;
    }
    if (strcmp(cmd, "nodeget") == 0)
    {
        CDC_HandleCanLikeCommand(0x0EU, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "nodeset") == 0)
    {
        if (CDC_ParseU8(arg1, &bytes[0]))
        {
            CDC_HandleCanLikeCommand(0x0FU, ParamId_GetCanNodeId(), bytes, 1U);
        }
        else
        {
            CDC_SendString("DBG ERR bad node\r\n");
        }
        return;
    }
    if (strcmp(cmd, "flashread") == 0)
    {
        CDC_HandleCanLikeCommand(0x0CU, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "flashclear") == 0)
    {
        CDC_HandleCanLikeCommand(0x0DU, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "calib") == 0)
    {
        if (CDC_ParseU8(arg1, &bytes[0]))
        {
            CDC_HandleCanLikeCommand(0x0AU, ParamId_GetCanNodeId(), bytes, 1U);
        }
        else
        {
            CDC_SendString("DBG ERR bad calib\r\n");
        }
        return;
    }
    if (strcmp(cmd, "calibstop") == 0)
    {
        CDC_HandleCanLikeCommand(0x0BU, ParamId_GetCanNodeId(), NULL, 0U);
        return;
    }
    if (strcmp(cmd, "can") == 0)
    {
        if ((CDC_ParseU8(arg1, &func)) && (CDC_ParseU8(arg2, &node)))
        {
            if (arg3 == NULL)
            {
                CDC_HandleCanLikeCommand(func, node, NULL, 0U);
            }
            else if (CDC_ParseHexBytes(arg3, bytes, sizeof(bytes), &byteLen))
            {
                CDC_HandleCanLikeCommand(func, node, bytes, byteLen);
            }
            else
            {
                CDC_SendString("DBG ERR bad payload\r\n");
            }
        }
        else
        {
            CDC_SendString("DBG ERR bad can args\r\n");
        }
        return;
    }

    CDC_SendString("DBG ERR unknown command\r\n");
}

void CDC_Debug_Service(void)
{
    if (s_cdc_rx.ready != 0U)
    {
        CDC_HandleLine(s_cdc_rx.line);
        s_cdc_rx.len = 0U;
        s_cdc_rx.ready = 0U;
    }

    if ((s_cdc_telem.valid != 0U) && ((HAL_GetTick() - s_cdc_telem.last_tick_ms) >= CDC_TELEM_PERIOD_MS))
    {
        CDC_SendTelemetryLine();
        s_cdc_telem.last_tick_ms = HAL_GetTick();
    }
}

void CDC_Debug_TaskHook(void)
{
    CDC_Debug_Service();
}
