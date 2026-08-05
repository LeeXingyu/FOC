/*
 * usart_cli.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#include "Communication/usart_cli.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "curr_autotune.h"
#include "mc_interface.h"
#include "mc_tasks.h"
#include "motor_control.h"
#include "param_identify.h"
#include "speed_autotune.h"
#include "Utils/Pid/pidregdqx_current.h"

osThreadId_t s_commTaskHandle = NULL;

uint8_t g_uartDmaBuf[UART3_DMA_BUF_SIZE];
char g_uartRxBuffer[UART3_DMA_BUF_SIZE];
uint8_t g_uUartRxLen = 0U;

static void Start_Motor(float fVal);
static void Stop_Motor(float fVal);
static void Set_Curr_Kp(float fVal);
static void Set_Curr_Ki(float fVal);
static void Set_Speed_Kp(float fVal);
static void Set_Speed_Ki(float fVal);
static void Set_Ref_Speed(float fVal);
static void Set_Speed_Model(float fVal);
static void Set_Position_Model(float fVal);
static void Set_Curr_Model(float fVal);
static void Set_VF_Model(float fVal);
static void Clear_Fault(float fVal);
static void Start_Current_AutoTune(float fVal);
static void Start_Speed_AutoTune(float fVal);
static void Get_Node_Id(float fVal);
static void Set_Node_Id(float fVal);
static void Start_Param_Calib(float fVal);
static void Stop_Param_Calib(float fVal);
static void Read_Flash(float fVal);
static void Clear_Flash(float fVal);

static const CmdTable cmdList[] = {
    {"start", Start_Motor},
    {"stop", Stop_Motor},
    {"cp", Set_Curr_Kp},
    {"ci", Set_Curr_Ki},
    {"sp", Set_Speed_Kp},
    {"si", Set_Speed_Ki},
    {"kp", Set_Speed_Kp},
    {"ki", Set_Speed_Ki},
    {"sc", Set_Ref_Speed},
    {"speed", Set_Ref_Speed},
    {"currmodel", Set_Curr_Model},
    {"speedmodel", Set_Speed_Model},
    {"speedmode", Set_Speed_Model},
    {"posimodel", Set_Position_Model},
    {"posmode", Set_Position_Model},
    {"vfmodel", Set_VF_Model},
    {"vfmode", Set_VF_Model},
    {"clearfault", Clear_Fault},
    {"ratune", Start_Current_AutoTune},
    {"satune", Start_Speed_AutoTune},
    {"nodeget", Get_Node_Id},
    {"nodeset", Set_Node_Id},
    {"calib", Start_Param_Calib},
    {"calibstop", Stop_Param_Calib},
    {"flashread", Read_Flash},
    {"flashclear", Clear_Flash},
};

#define CMD_COUNT (sizeof(cmdList) / sizeof(cmdList[0]))

void Uart_Protocol_Init(void)
{
    /* DMA/IDLE setup is handled by the CubeMX-generated UART code. */
}

void Uart_IDLE_Callback(void)
{
    if (s_commTaskHandle != NULL)
    {
        osThreadFlagsSet(s_commTaskHandle, NOTIFY_RX_DONE);
    }
}

static float Simple_Atof(const char *pStr)
{
    float fResult = 0.0f;
    float fFactor = 1.0f;
    int8_t iSign = 1;

    if (*pStr == '-')
    {
        iSign = -1;
        pStr++;
    }
    else if (*pStr == '+')
    {
        pStr++;
    }

    while ((*pStr >= '0') && (*pStr <= '9'))
    {
        fResult = (fResult * 10.0f) + (float)(*pStr++ - '0');
    }

    if (*pStr == '.')
    {
        pStr++;
        while ((*pStr >= '0') && (*pStr <= '9'))
        {
            fFactor *= 0.1f;
            fResult += (float)(*pStr++ - '0') * fFactor;
        }
    }

    return (float)iSign * fResult;
}

void Uart_Parse_Command(uint8_t *pBuf, uint16_t uLen, char *pCmd, float *fVal)
{
    uint16_t uCmdLen = 0U;

    if ((pBuf == NULL) || (pCmd == NULL) || (fVal == NULL) || (uLen == 0U))
    {
        return;
    }

    while ((uCmdLen < uLen) && (pBuf[uCmdLen] == ' '))
    {
        uCmdLen++;
    }

    while ((uCmdLen < uLen) &&
           (pBuf[uCmdLen] != '\0') &&
           (((pBuf[uCmdLen] >= 'a') && (pBuf[uCmdLen] <= 'z')) ||
            ((pBuf[uCmdLen] >= 'A') && (pBuf[uCmdLen] <= 'Z'))))
    {
        uCmdLen++;
    }

    if (uCmdLen == 0U)
    {
        return;
    }

    (void)memcpy(pCmd, pBuf, uCmdLen);
    pCmd[uCmdLen] = '\0';
    *fVal = 0.0f;

    if ((uCmdLen + 1U) < uLen)
    {
        char strTemp[20] = {0};
        uint16_t uValueLen = (uint16_t)(uLen - uCmdLen - 1U);

        if (uValueLen >= sizeof(strTemp))
        {
            uValueLen = (uint16_t)(sizeof(strTemp) - 1U);
        }

        (void)memcpy(strTemp, pBuf + uCmdLen + 1U, uValueLen);
        strTemp[uValueLen] = '\0';
        *fVal = Simple_Atof(strTemp);
    }
}

void Uart_Cmd_Dispatch(const char *pCmd, float fVal)
{
    uint32_t i;

    if (pCmd == NULL)
    {
        return;
    }

    for (i = 0U; i < CMD_COUNT; i++)
    {
        if (strcmp(pCmd, cmdList[i].cmd) == 0)
        {
            cmdList[i].handler(fVal);
            return;
        }
    }

    printf("[CLI] unknown cmd: %s\r\n", pCmd);
}

static void Start_Motor(float fVal)
{
    (void)fVal;
    if (g_axis.state == AXIS_STATE_IDLE)
    {
        (void)MC_Start_Motor();
    }
}

static void Stop_Motor(float fVal)
{
    (void)fVal;
    (void)MC_Stop_Motor();
    (void)MC_Calib_StopParam();
    CurrAutoTune_Abort();
    SpeedAutoTune_Abort();
}

static void Set_Curr_Kp(float fVal)
{
    PIDREGDQX_CURRENT_setKp_si(&g_axis.currCtrl.pid_IdIqX_obj, fVal);
    printf("[CLI] current Kp=%.6f\r\n", fVal);
}

static void Set_Curr_Ki(float fVal)
{
    PIDREGDQX_CURRENT_setWi_si(&g_axis.currCtrl.pid_IdIqX_obj, fVal);
    printf("[CLI] current Wi=%.6f\r\n", fVal);
}

static void Set_Speed_Kp(float fVal)
{
    MC_Set_Speed_Kp(fVal);
    printf("[CLI] speed Kp=%.6f\r\n", fVal);
}

static void Set_Speed_Ki(float fVal)
{
    MC_Set_Speed_Ki(fVal);
    printf("[CLI] speed Ki=%.6f\r\n", fVal);
}

static void Set_Ref_Speed(float fVal)
{
    MC_Set_Speed_Reference(fVal);
    printf("[CLI] speed ref=%.6f rpm\r\n", fVal);
}

static void Set_Speed_Model(float fVal)
{
    (void)fVal;
    MC_Set_Control_Mode(CTRL_MODE_SPEED);
}

static void Set_Position_Model(float fVal)
{
    (void)fVal;
    g_axis.posCtrl.traj.bEnable = true;
    MC_Set_Control_Mode(CTRL_MODE_POSITION);
}

static void Set_Curr_Model(float fVal)
{
    (void)fVal;
    MC_Set_Control_Mode(CTRL_MODE_TORQUE);
}

static void Set_VF_Model(float fVal)
{
    (void)fVal;
    MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
}

static void Clear_Fault(float fVal)
{
    (void)fVal;
    (void)MC_Fault_Reset();
}

static void Start_Current_AutoTune(float fVal)
{
	(void)fVal;
	if (g_axis.state != AXIS_STATE_IDLE)
	{
        printf("[CLI] current autotune rejected, state=%u\r\n", (unsigned)g_axis.state);
        return;
    }

	if (MC_Start_Motor() != MC_SUCCESS)
	{
		printf("[CLI] current autotune start failed\r\n");
		return;
	}

	CurrAutoTune_Start();
	g_bStartSpeedAutoTune = false;
	g_bStartCurrentAutoTune = true;
}

static void Start_Speed_AutoTune(float fVal)
{
    float testCurrentA = fVal;

    if (testCurrentA <= 0.0f)
    {
        testCurrentA = SPEED_AUTOTUNE_DEFAULT_CURRENT_A;
    }

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        printf("[CLI] speed autotune rejected, state=%u\r\n", (unsigned)g_axis.state);
        return;
    }

	if (MC_Start_Motor() != MC_SUCCESS)
	{
		printf("[CLI] speed autotune start failed\r\n");
		return;
	}

	SpeedAutoTune_Start(testCurrentA);
	CurrAutoTune_Start();
	g_bStartSpeedAutoTune = true;
	g_bStartCurrentAutoTune = true;
}

static void Get_Node_Id(float fVal)
{
    (void)fVal;
    printf("[CLI] node id=%u\r\n", (unsigned)ParamId_GetCanNodeId());
}

static void Set_Node_Id(float fVal)
{
    uint8_t nodeId = (uint8_t)((fVal < 0.0f) ? 0.0f : fVal);
    if (ParamId_SaveCanNodeIdToFlash(nodeId))
    {
        printf("[CLI] node id saved=%u\r\n", (unsigned)nodeId);
    }
    else
    {
        printf("[CLI] node id save failed=%u\r\n", (unsigned)nodeId);
    }
}

static void Start_Param_Calib(float fVal)
{
    MC_RetStatus_t ret;
    ParamIdStep_t mode = (fVal < 0.5f) ? PARAM_ID_STEP_RS : PARAM_ID_STEP_ALL;

    if (g_axis.state != AXIS_STATE_IDLE)
    {
        printf("[CLI] calib rejected, state=%u\r\n", (unsigned)g_axis.state);
        return;
    }

    if (mode == PARAM_ID_STEP_RS)
    {
        ret = MC_Calib_StartChain();
        printf("[CLI] calib chain start\r\n");
    }
    else
    {
        ret = MC_Calib_StartParam(PARAM_ID_STEP_ALL);
        printf("[CLI] calib full start\r\n");
    }

    if (ret != MC_SUCCESS)
    {
        printf("[CLI] calib start failed\r\n");
    }
}

static void Stop_Param_Calib(float fVal)
{
    (void)fVal;
    if (MC_Calib_StopParam() != MC_SUCCESS)
    {
        printf("[CLI] calib stop failed\r\n");
    }
}

static void Read_Flash(float fVal)
{
    ParamIdFlashData_t data;

    (void)fVal;
    memset(&data, 0, sizeof(data));
    if (!ParamId_LoadFromFlash(&data))
    {
        printf("[CLI] flash read failed\r\n");
        return;
    }

    (void)ParamId_ApplyFlashDataToAxis(&data);
    printf("[CLI] flash read ok: node=%u polePairs=%lu currKp=%.6f currWi=%.6f\r\n",
           (unsigned)data.can_node_id,
           (unsigned long)data.pole_pairs,
           data.curr_kp_si,
           data.curr_wi_si);
}

static void Clear_Flash(float fVal)
{
    (void)fVal;
    if (ParamId_ClearFlash())
    {
        printf("[CLI] flash cleared\r\n");
    }
    else
    {
        printf("[CLI] flash clear failed\r\n");
    }
}
