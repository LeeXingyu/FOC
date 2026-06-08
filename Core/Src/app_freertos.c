/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "tim.h"
#include "motor_parameters.h"
#include "tem_task.h"
#include "canopen.h"
#include "ethercat.h"
#include "Communication/mcp2518fd/can_telemetry.h"
#include "MotorControl/Tasks/mc_tasks.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint8_t g_comm_io1_irq_pending = 0U;
volatile uint8_t g_comm_io2_irq_pending = 0U;
volatile uint8_t g_comm_int_irq_pending = 0U;
volatile uint8_t g_mc_high_freq_busy = 0U;
volatile uint8_t g_mc_medium_freq_busy = 0U;
volatile uint32_t g_rtos_task_create_fail_mask = 0U;

osThreadId_t mcHighFreqTaskHandle = NULL;
osThreadId_t mcMediumFreqTaskHandle = NULL;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Comm_Task */
osThreadId_t Comm_TaskHandle;
const osThreadAttr_t Comm_Task_attributes = {
  .name = "Comm_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void RTOS_MarkTaskCreateFailure(uint32_t bit);

const osThreadAttr_t mcHighFreqTask_attributes = {
  .name = "mcHighFreqTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};

const osThreadAttr_t mcMediumFreqTask_attributes = {
  .name = "mcMediumFreqTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

osThreadId_t temTaskHandle;
const osThreadAttr_t temTask_attributes = {
  .name = "temTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Communication_Task(void *argument);

static void RTOS_MarkTaskCreateFailure(uint32_t bit)
{
  g_rtos_task_create_fail_mask |= bit;
}

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  if (defaultTaskHandle == NULL)
  {
    RTOS_MarkTaskCreateFailure(1UL << 0);
  }
  /* creation of Comm_Task */
  Comm_TaskHandle = osThreadNew(Communication_Task, NULL, &Comm_Task_attributes);
  if (Comm_TaskHandle == NULL)
  {
    RTOS_MarkTaskCreateFailure(1UL << 1);
  }
  temTaskHandle = osThreadNew(Tem_Task, NULL, &temTask_attributes);
  if (temTaskHandle == NULL)
  {
    RTOS_MarkTaskCreateFailure(1UL << 2);
  }
  /* creation of control tasks */
  mcHighFreqTaskHandle = osThreadNew(StartHighFrequencyTask, NULL, &mcHighFreqTask_attributes);
  if (mcHighFreqTaskHandle == NULL)
  {
    RTOS_MarkTaskCreateFailure(1UL << 3);
  }
  mcMediumFreqTaskHandle = osThreadNew(StartMediumFrequencyTask, NULL, &mcMediumFreqTask_attributes);
  if (mcMediumFreqTaskHandle == NULL)
  {
    RTOS_MarkTaskCreateFailure(1UL << 4);
  }


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start the control timers only after the task handles exist so IRQs can notify safely. */
  (void)HAL_TIM_Base_Start_IT(&htim1);
  (void)HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  // MX_USB_Device_Init();
  for(;;)
  {
    osDelay(1);
  }
}
/* USER CODE BEGIN Header_Communication_Task */
/**
* @brief Function implementing the Comm_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communication_Task */
void Communication_Task(void *argument)
{
  /* USER CODE BEGIN Communication_Task */
  /* Infinite loop */
  for(;;)
  {
    if (g_system_comm_mode == COMM_PROTO_CAN)
    {
      if (g_comm_int_irq_pending != 0U)
      {
        g_comm_int_irq_pending = 0U;
        MCP2518FD_ProcessRxIrq();
      }

      if (g_comm_io1_irq_pending != 0U)
      {
        g_comm_io1_irq_pending = 0U;
      }

      if (g_comm_io2_irq_pending != 0U)
      {
        g_comm_io2_irq_pending = 0U;
      }

      MCP2518FD_Service1ms();
      CAN_Telemetry_Service1ms();
    }
    else if (g_system_comm_mode == COMM_PROTO_ETHERCAT)
    {
      LAN9253_Process();
    }

    osDelay(1);
  }
  /* USER CODE END Communication_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

