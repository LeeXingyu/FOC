/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv8353rs.h"
#include "motor_control.h"
#include "motor_parameters.h"
#include "Communication/ethercat.h"
#include "Communication/mcp2518fd/can_telemetry.h"
#include "usbd_cdc_if.h"
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

/* USER CODE BEGIN PV */
Axis_t g_axis;
uint8_t uart_data_ready = 0;
ADC_Rule_Data_t g_adc_Rule_ID_Tem;
Comm_Protocol_t g_system_comm_mode = COMM_PROTO_UNKNOWN;
//extern char g_uartRxBuffer[UART3_DMA_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  SPI1_SwitchMode(SPI_MODE_KTH7824);
  // 电机初始化
  CDC_Transmit_FS ((uint8_t *)"Hello\r",6);
  Motor_Control_Init();

  // 驱动板初始化
  DRV8353RS_Init();

  ADC_Rule_Collect(&hadc2,&g_adc_Rule_ID_Tem);

  if(g_system_comm_mode == COMM_PROTO_CAN)
  {
	  CANFD_INIT();
	  CAN_Telemetry_Init();
  }
  else if(g_system_comm_mode == COMM_PROTO_ETHERCAT)
  {
	  LAN9253_Init();
  }
  if(SPI_MODE_KTH7824)
  {

  }

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ADC_Rule_Collect(ADC_HandleTypeDef* hadc, ADC_Rule_Data_t* data)
{
	uint16_t raw_val = 0;
    // 1. 启动 ADC 规则组转换
    if (HAL_ADC_Start(hadc) != HAL_OK)
    {
        // 启动失败处理
        return;
    }

    // 2. 依次获取 4 个通道的数据
    // 注意：在单次扫描模式下，HAL 库会自动按 Rank 顺序排列
    // 我们只需要按顺序调用 Get_Value 即可

    // --- Rank 1: Channel 17 (COMMID) ---
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) // 等待转换完成，超时10ms
    {
    	raw_val = HAL_ADC_GetValue(hadc) >> 4;
    	if(raw_val > 2000)
    	{
    		data->raw_COMM_ID = 1;

    	}
    	else
    	{
    		data->raw_COMM_ID = 0;
    	}
    	g_system_comm_mode = data->raw_COMM_ID;

    }
    //需要根据热敏电阻系数进行计算得出实际温度
    // --- Rank 2: Channel 12 ---
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        data->raw_TSENA = HAL_ADC_GetValue(hadc);
    }

    // --- Rank 3: Channel 5 ---
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        data->raw_TSENB = HAL_ADC_GetValue(hadc);
    }

    // --- Rank 4: Channel 11 ---
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK)
    {
        data->raw_TSENC = HAL_ADC_GetValue(hadc);
    }

    // 3. 停止 ADC (可选，但建议加上以省电和确保状态清晰)
    HAL_ADC_Stop(hadc);
}

void SPI1_SwitchMode(uint8_t mode)
{
    // 1. 先禁用 SPI 外设，否则无法修改配置
    __HAL_SPI_DISABLE(&hspi3);

    // 2. 根据模式修改 Init 结构体参数
    if (mode == SENSOR_MODE_A)
    {
        // AS5047P 需要 Mode 1
    	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;  // 空闲时低电平
    	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;      // 第二个边沿采样
    }
    else if (mode == SENSOR_MODE_K)
    {
        // KTH7824 需要 Mode 3
    	hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH; // 空闲时高电平
    	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;      // 第二个边沿采样
    }

    // 3. 重新初始化 SPI 以应用更改
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        // 初始化失败处理 (可选)
        Error_Handler();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == DRV_FAULT_N_Pin)
	{
		g_axis.error = (AxisError_t)(g_axis.error | AXIS_ERROR_GATE_DRIVER);
		g_axis.state = AXIS_STATE_FAULT_NOW;
		return;
	}

	if (GPIO_Pin == COMM_IO1_Pin)
	{
		g_comm_io1_irq_pending = 1U;
		return;
	}

	if (GPIO_Pin == COMM_IO2_Pin)
	{
		g_comm_io2_irq_pending = 1U;
		return;
	}

	if (GPIO_Pin == COMM_INT_Pin)
	{
		g_comm_int_irq_pending = 1U;
		return;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
