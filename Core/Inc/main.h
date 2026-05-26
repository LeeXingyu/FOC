/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mc_type.h"
#include "canopen.h"
#include "spi_switch.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern Axis_t g_axis;
// 定义存放ADC数据的结构体
typedef struct {
    uint8_t raw_COMM_ID;    // 对应 adc2 Rank 1 (Channel 17)
    uint16_t raw_TSENA;     // 对应 adc2 Rank 2 (Channel 12)
    uint16_t raw_TSENB;     // 对应 adc2 Rank 3 (Channel 5)
    uint16_t raw_TSENC;     // 对应 adc2 Rank 4 (Channel 11)

    // 如果需要，可以在这里加计算后的浮点值
    // float temperature_c;
} ADC_Rule_Data_t;

// 实例化一个全局变量（或者在 main 中定义传递指针）
extern ADC_Rule_Data_t g_adc_Rule_ID_Tem;

// 1. 定义通信协议的枚举类型
typedef enum {
    COMM_PROTO_CAN = 0,       // 对应 ADC ID 0
    COMM_PROTO_ETHERCAT = 1,  // 对应 ADC ID 1
    COMM_PROTO_UNKNOWN = 2    // 容错处理
} Comm_Protocol_t;

// 2. 定义一个全局变量来存储当前的通信状态
// 加上 volatile 防止编译器过度优化，虽然这里主要是初始化赋值
extern Comm_Protocol_t g_system_comm_mode;
extern volatile uint8_t g_comm_io1_irq_pending;
extern volatile uint8_t g_comm_io2_irq_pending;
extern volatile uint8_t g_comm_int_irq_pending;

// 定义 SPI 模式
typedef enum {
    SENSOR_MODE_A = 1, // AS5047P 使用 Mode 1
    SENSOR_MODE_K = 3  // KTH7824 使用 Mode 3
} SensorMode_t;
#define KTH7824
#ifdef KTH7824
#define SPI_MODE_KTH7824  3  // Mode 3: CPOL=1, CPHA=1 (2 Edge)
#else
#define AS5047P
#define SPI_MODE_AS5047P  1  // Mode 1: CPOL=0, CPHA=1 (2 Edge)
#endif
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRV_INLC_Pin GPIO_PIN_13
#define DRV_INLC_GPIO_Port GPIOC
#define MCU_DO1_Pin GPIO_PIN_14
#define MCU_DO1_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOC
#define COMM_IO2_Pin GPIO_PIN_0
#define COMM_IO2_GPIO_Port GPIOC
#define COMM_IO2_EXTI_IRQn EXTI0_IRQn
#define CODER_CS_N2_Pin GPIO_PIN_1
#define CODER_CS_N2_GPIO_Port GPIOC
#define VSENB_Pin GPIO_PIN_2
#define VSENB_GPIO_Port GPIOC
#define VSENC_Pin GPIO_PIN_3
#define VSENC_GPIO_Port GPIOC
#define ISENC_Pin GPIO_PIN_0
#define ISENC_GPIO_Port GPIOA
#define ISENB_Pin GPIO_PIN_1
#define ISENB_GPIO_Port GPIOA
#define ISENA_Pin GPIO_PIN_2
#define ISENA_GPIO_Port GPIOA
#define COMM_ID_Pin GPIO_PIN_4
#define COMM_ID_GPIO_Port GPIOA
#define CODER2_SCLK_Pin GPIO_PIN_5
#define CODER2_SCLK_GPIO_Port GPIOA
#define CODER2_MISO_Pin GPIO_PIN_6
#define CODER2_MISO_GPIO_Port GPIOA
#define CODER2_MOSI_Pin GPIO_PIN_7
#define CODER2_MOSI_GPIO_Port GPIOA
#define TSENB_Pin GPIO_PIN_4
#define TSENB_GPIO_Port GPIOC
#define TSENC_Pin GPIO_PIN_5
#define TSENC_GPIO_Port GPIOC
#define DRV_INLB_Pin GPIO_PIN_0
#define DRV_INLB_GPIO_Port GPIOB
#define VSENVM_Pin GPIO_PIN_1
#define VSENVM_GPIO_Port GPIOB
#define TSENA_Pin GPIO_PIN_2
#define TSENA_GPIO_Port GPIOB
#define COMM_RST_Pin GPIO_PIN_10
#define COMM_RST_GPIO_Port GPIOB
#define VSENA_Pin GPIO_PIN_11
#define VSENA_GPIO_Port GPIOB
#define COMM_CS_N_Pin GPIO_PIN_12
#define COMM_CS_N_GPIO_Port GPIOB
#define COMM_SCK_Pin GPIO_PIN_13
#define COMM_SCK_GPIO_Port GPIOB
#define COMM_MISO_Pin GPIO_PIN_14
#define COMM_MISO_GPIO_Port GPIOB
#define COMM_MOSI_Pin GPIO_PIN_15
#define COMM_MOSI_GPIO_Port GPIOB
#define BRAKE_PWM_Pin GPIO_PIN_9
#define BRAKE_PWM_GPIO_Port GPIOC
#define DRV_NHC_Pin GPIO_PIN_8
#define DRV_NHC_GPIO_Port GPIOA
#define DRV_INHB_Pin GPIO_PIN_9
#define DRV_INHB_GPIO_Port GPIOA
#define DRV_INHA_Pin GPIO_PIN_10
#define DRV_INHA_GPIO_Port GPIOA
#define COMM_IO1_Pin GPIO_PIN_15
#define COMM_IO1_GPIO_Port GPIOA
#define COMM_IO1_EXTI_IRQn EXTI15_10_IRQn
#define COMM_INT_Pin GPIO_PIN_10
#define COMM_INT_GPIO_Port GPIOC
#define COMM_INT_EXTI_IRQn EXTI15_10_IRQn
#define MCU_DI1_Pin GPIO_PIN_11
#define MCU_DI1_GPIO_Port GPIOC
#define CODER_CS_N1_Pin GPIO_PIN_12
#define CODER_CS_N1_GPIO_Port GPIOC
#define DRV_CS_N_Pin GPIO_PIN_2
#define DRV_CS_N_GPIO_Port GPIOD
#define CODER_SCK_Pin GPIO_PIN_3
#define CODER_SCK_GPIO_Port GPIOB
#define CODER_MISO_Pin GPIO_PIN_4
#define CODER_MISO_GPIO_Port GPIOB
#define CODER_MOSI_Pin GPIO_PIN_5
#define CODER_MOSI_GPIO_Port GPIOB
#define DRV_EN_Pin GPIO_PIN_6
#define DRV_EN_GPIO_Port GPIOB
#define DRV_FAULT_N_Pin GPIO_PIN_7
#define DRV_FAULT_N_GPIO_Port GPIOB
#define DRV_FAULT_N_EXTI_IRQn EXTI9_5_IRQn
#define DRV_INLA_Pin GPIO_PIN_9
#define DRV_INLA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void ADC_Rule_Collect(ADC_HandleTypeDef* hadc, ADC_Rule_Data_t* data);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
