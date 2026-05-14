/*
 * usart_cli.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#include "Communication/usart_cli.h"
#include "main.h"
//#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "mc_interface.h"


//extern UART_HandleTypeDef huart3;

osThreadId_t s_commTaskHandle = NULL;		// 线程标志，通知指令解析

uint8_t g_uartDmaBuf[UART3_DMA_BUF_SIZE];   // DMA 接收缓冲区（持续接收）
char g_uartRxBuffer[UART3_DMA_BUF_SIZE];	// 处理用的缓冲区
uint8_t g_uUartRxLen = 0;             		// 本次收到的字节数

/**
  * @brief  启动电机
  */
static void Start_Motor(float fVal);
/**
  * @brief  停止电机
  */
static void Stop_Motor(float fVal);
/**
  * @brief  设置电流环kp
  */
static void Set_Curr_Kp(float fVal);
/**
  * @brief  设置电流环ki
  */
static void Set_Curr_Ki(float fVal);
/**
  * @brief  设置速度环kp
  */
static void Set_Speed_Kp(float fVal);
/**
  * @brief  设置速度环ki
  */
static void Set_Speed_Ki(float fVal);
/**
  * @brief  设置目标速度
  */
static void Set_Ref_Speed(float fVal);
/**
  * @brief  设置速度模式
  */
static void Set_Speed_Model(float fVal);
/**
  * @brief  设置位置模式
  */
static void Set_Position_Model(float fVal);
/**
  * @brief  设置V/F拖动（开环模式）
  */
static void Set_VF_Model(float fVal);

// 指令、处理函数列表
static const CmdTable cmdList[] = {
    {"start", Start_Motor},
    {"stop", Stop_Motor},
    {"sp", Set_Speed_Kp},
    {"si", Set_Speed_Ki},
    {"sc", Set_Ref_Speed},
    {"vfmodel", Set_VF_Model},
    {"speedmodel", Set_Speed_Model}
};
// 指令个数
#define CMD_COUNT  (sizeof(cmdList) / sizeof(cmdList[0]))

/**
  * @brief  串口初始化（dma）
  */
//void Uart_Protocol_Init(void)
//{
//	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);          				// 开启 IDLE 中断
//	HAL_UART_Receive_DMA(&huart3, g_uartDmaBuf, UART3_DMA_BUF_SIZE);  	// 启动 DMA 接收
//}

/**
  * @brief  空闲中断回调，通知处理解析指令
  */
void Uart_IDLE_Callback(void)
{
	// 通知 CommTask 处理
	if (s_commTaskHandle != NULL)
	{
		osThreadFlagsSet(s_commTaskHandle, NOTIFY_RX_DONE);
	}
}

/**
  * @brief  解析字符串
  */
static float Simple_Atof(const char *pStr)
{
    float fResult = 0.0f;
    float fFactor = 1.0f;
    int8_t iSign = 1;

    if (*pStr == '-') { iSign = -1; pStr++; }
    else if (*pStr == '+') { pStr++; }

    while (*pStr >= '0' && *pStr <= '9')
    	fResult = fResult * 10.0f + (*pStr++ - '0');

    if (*pStr == '.')
    {
    	pStr++;
        while (*pStr >= '0' && *pStr <= '9') {
        	fFactor *= 0.1f;
        	fResult += (*pStr++ - '0') * fFactor;
        }
    }
    return iSign * fResult;
}

/**
  * @brief  指令解析
  */
void Uart_Parse_Command(uint8_t *pBuf, uint16_t uLen, char *pCmd, float *fVal)
{
	uint8_t uCmdLen = 0;

	// 跳过前导空格
	while (pBuf[uCmdLen] == ' ')
	{
		uCmdLen++;
	}

	// 提取命令字符串（字母部分）
	uint8_t uCmdStart = uCmdLen;
	while (pBuf[uCmdLen] != '\0' &&
	   ((pBuf[uCmdLen] >= 'a' && pBuf[uCmdLen] <= 'z') ||
		(pBuf[uCmdLen] >= 'A' && pBuf[uCmdLen] <= 'Z')))
	{
		uCmdLen++;
	}

	if (uCmdLen == 0 || uCmdLen >= uLen)
	{
		return;  // 命令为空或太长
	}

	// 拷贝命令字符串
	strncpy(pCmd, pBuf, uCmdLen);
	pCmd[uCmdLen] = '\0';

	if (uLen - 1 != uCmdLen)
	{
		char strTemp[20] = {0};
		uint8_t uValueLen = uLen - uCmdLen - 2;

		strncpy(strTemp, pBuf + uCmdLen + 1, uValueLen);
		strTemp[uValueLen] = '\0';

		*fVal = (float)Simple_Atof(strTemp);

	}
}

void Uart_Cmd_Dispatch(const char *pCmd, float fVal)
{
	for (int i = 0; i < CMD_COUNT; i++)
	{
		if (strcmp(pCmd, cmdList[i].cmd) == 0)
		{
			cmdList[i].handler(fVal);
			return;
		}
	}

	return;
}

void Start_Motor(float fVal)
{
	if (g_axis.state == AXIS_STATE_IDLE)
	{
		MC_Start_Motor();
	}
}

void Stop_Motor(float fVal)
{
	MC_Stop_Motor();
}

void Set_Curr_Kp(float fVal)
{

}

void Set_Curr_Ki(float fVal)
{

}

void Set_Speed_Kp(float fVal)
{
	MC_Set_Speed_Kp(fVal);
}

void Set_Speed_Ki(float fVal)
{
	MC_Set_Speed_Ki(fVal);
}

void Set_Ref_Speed(float fVal)
{
	MC_Set_Speed_Reference(fVal);
}

void Set_Speed_Model(float fVal)
{
	MC_Set_Control_Mode(CTRL_MODE_SPEED);
}

void Set_Position_Model(float fVal)
{
	MC_Set_Control_Mode(CTRL_MODE_POSITION);
}

void Set_VF_Model(float fVal)
{
	MC_Set_Control_Mode(CTRL_MODE_OPEN_LOOP);
}

