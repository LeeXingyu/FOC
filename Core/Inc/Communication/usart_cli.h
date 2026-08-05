/*
 * usart_cli.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Administrator
 */

#ifndef INC_COMMUNICATION_USART_CLI_H_
#define INC_COMMUNICATION_USART_CLI_H_

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

#define UART3_DMA_BUF_SIZE     64
#define VOFA_SEND_INTERVAL_MS  10
#define NOTIFY_RX_DONE        0x01U

extern uint8_t g_uartDmaBuf[UART3_DMA_BUF_SIZE];
extern char g_uartRxBuffer[UART3_DMA_BUF_SIZE];
extern uint8_t g_uUartRxLen;
extern osThreadId_t s_commTaskHandle;

typedef void (*CmdHandler)(float fVaule);

typedef struct
{
    const char *cmd;
    CmdHandler handler;
} CmdTable;

void Uart_Protocol_Init(void);
void Uart_IDLE_Callback(void);
void Uart_Parse_Command(uint8_t *pBuf, uint16_t uLen, char *pCmd, float *fVal);
void Uart_Cmd_Dispatch(const char *pCmd, float fVal);

#endif /* INC_COMMUNICATION_USART_CLI_H_ */
