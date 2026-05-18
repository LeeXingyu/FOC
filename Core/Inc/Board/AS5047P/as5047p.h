/*
 * as5047p.h
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */

#ifndef INC_BOARD_AS5047P_AS5047P_H_
#define INC_BOARD_AS5047P_AS5047P_H_

#include <stdbool.h>
#include <stdint.h>
#include "spi.h"
#include "main.h"
#include "gpio.h"

#define AS5047P_NOP_REG        0x0000U
#define AS5047P_ERRFL_REG      0x0001U
#define AS5047P_DIAAGC_REG     0x3FFCU
#define AS5047P_MAG_REG        0x3FFDU
#define AS5047P_ANGLEUNC_REG   0x3FFEU
#define AS5047P_ANGLECOM_REG   0x3FFFU

#define AS5047P_READ_CMD       0x4000U
#define AS5047P_WRITE_CMD      0x0000U
#define AS5047P_DATA_MASK      0x3FFFU
#define AS5047P_COUNT          16384U

#define AS5047P_ERR_PARERR     0x0004U
#define AS5047P_ERR_INVCOMM    0x0002U
#define AS5047P_ERR_FRERR      0x0001U

#define AS5047P_DIAAGC_FIELD_LOW   0x0800U
#define AS5047P_DIAAGC_FIELD_HIGH  0x0400U
#define AS5047P_DIAAGC_COF     0x0200U
#define AS5047P_DIAAGC_LF      0x0100U

typedef struct {
  uint16_t raw_angle;       /* Compensated angle, recommended for FOC. */
  uint16_t raw_unc_angle;   /* Uncompensated angle, useful for diagnostics. */
  float angle_deg;          /* Mechanical angle in degrees. */
  float angle_pu;           /* Mechanical angle per-unit: 0.0 to 1.0. */
  float electrical_pu;      /* Electrical angle per-unit: 0.0 to 1.0. */
  float speed_rpm;          /* Mechanical speed calculated from angle delta. */
  uint16_t agc_value;       /* Automatic gain control value. */
  uint16_t mag_value;       /* CORDIC magnitude. */
  uint16_t errfl;           /* Error flag register. */
  uint16_t diaagc;          /* Diagnostic and AGC register. */
  uint8_t error_flag;       /* SPI/frame/magnetic diagnostic error. */
  uint8_t mag_too_low;
  uint8_t mag_too_high;
  uint8_t cordic_overflow;
  uint8_t offset_comp_finished;
} AS5047P_Data;

#define CODER_CS_LOW_1()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define CODER_CS_HIGH_1()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define CODER_CS_LOW_2()    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET)
#define CODER_CS_HIGH_2()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET)

AS5047P_Data Read_AS5047P_Angle(uint8_t coder_num);
AS5047P_Data AS5047P_Read_FOC_Data(uint8_t coder_num, uint8_t pole_pairs, float sample_period_s);
uint16_t AS5047P_Read_RawAngle(uint8_t coder_num);
uint16_t AS5047P_Read_UncompensatedAngle(uint8_t coder_num);
uint8_t AS5047P_Has_Error(const AS5047P_Data *data);
void AS5047P_Check_Encoder_Errors(AS5047P_Data *data);

#endif /* INC_BOARD_AS5047P_AS5047P_H_ */
