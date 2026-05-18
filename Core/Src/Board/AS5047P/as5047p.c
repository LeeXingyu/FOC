/*
 * as5047p.c
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */
#include "as5047p.h"
#include "MotorControl/Core/motor_parameters.h"
#include <math.h>
#include <string.h>

#define AS5047P_FRAME_PARITY_BIT  0x8000U
#define AS5047P_FRAME_ERROR_BIT   0x4000U
#define AS5047P_DEFAULT_DT_S      (1.0f / (float)PWM_FREQUENCY)

typedef struct {
	uint16_t prev_angle;
	uint8_t initialized;
} AS5047P_SpeedState_t;

static AS5047P_SpeedState_t s_speed_state[2];

static uint8_t AS5047P_EvenParity(uint16_t value)
{
	value ^= value >> 8;
	value ^= value >> 4;
	value ^= value >> 2;
	value ^= value >> 1;
	return (uint8_t)(value & 0x1U);
}

static uint16_t AS5047P_BuildCommand(uint16_t reg_addr, bool read)
{
	uint16_t frame = (reg_addr & AS5047P_DATA_MASK);

	if (read)
	{
		frame |= AS5047P_READ_CMD;
	}
	else
	{
		frame |= AS5047P_WRITE_CMD;
	}

	if (AS5047P_EvenParity(frame) != 0U)
	{
		frame |= AS5047P_FRAME_PARITY_BIT;
	}

	return frame;
}

void AS5047P_Encoder_CS(uint8_t coder_num, uint8_t status)
{
	if (coder_num == 1U)
	{
		HAL_GPIO_WritePin(CODER_CS_N1_GPIO_Port, CODER_CS_N1_Pin,
		                  (status != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	else if (coder_num == 2U)
	{
		HAL_GPIO_WritePin(CODER_CS_N2_GPIO_Port, CODER_CS_N2_Pin,
		                  (status != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
	else
	{
		CODER_CS_HIGH_1();
		CODER_CS_HIGH_2();
	}
}

static uint16_t AS5047P_SPI_Transfer(uint16_t data)
{
	uint16_t rx_data = 0U;
	(void)HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&data, (uint8_t *)&rx_data, 1U, HAL_MAX_DELAY);
	return rx_data;
}

static uint16_t AS5047P_Read_Register(uint16_t reg_addr, uint8_t coder_num)
{
	uint16_t cmd = AS5047P_BuildCommand(reg_addr, true);
	uint16_t raw;

	AS5047P_Encoder_CS(coder_num, 0U);
	(void)AS5047P_SPI_Transfer(cmd);
	AS5047P_Encoder_CS(coder_num, 1U);

	AS5047P_Encoder_CS(coder_num, 0U);
	raw = AS5047P_SPI_Transfer(AS5047P_BuildCommand(AS5047P_NOP_REG, true));
	AS5047P_Encoder_CS(coder_num, 1U);

	return raw;
}

static uint16_t AS5047P_GetData(uint16_t raw_frame)
{
	return (uint16_t)(raw_frame & AS5047P_DATA_MASK);
}

static uint8_t AS5047P_FrameHasError(uint16_t raw_frame)
{
	uint8_t parity_error = (AS5047P_EvenParity(raw_frame) != 0U) ? 1U : 0U;
	uint8_t ef_error = ((raw_frame & AS5047P_FRAME_ERROR_BIT) != 0U) ? 1U : 0U;
	return (uint8_t)(parity_error | ef_error);
}

uint16_t AS5047P_Read_RawAngle(uint8_t coder_num)
{
	return AS5047P_GetData(AS5047P_Read_Register(AS5047P_ANGLECOM_REG, coder_num));
}

uint16_t AS5047P_Read_UncompensatedAngle(uint8_t coder_num)
{
	return AS5047P_GetData(AS5047P_Read_Register(AS5047P_ANGLEUNC_REG, coder_num));
}

static float AS5047P_CalcSpeedRpm(uint8_t coder_num, uint16_t angle, float sample_period_s)
{
	uint8_t idx = (coder_num > 0U) ? (uint8_t)(coder_num - 1U) : 0U;
	int32_t delta;

	if (idx >= (sizeof(s_speed_state) / sizeof(s_speed_state[0])))
	{
		idx = 0U;
	}

	if (sample_period_s <= 0.0f)
	{
		sample_period_s = AS5047P_DEFAULT_DT_S;
	}

	if (s_speed_state[idx].initialized == 0U)
	{
		s_speed_state[idx].prev_angle = angle;
		s_speed_state[idx].initialized = 1U;
		return 0.0f;
	}

	delta = (int32_t)angle - (int32_t)s_speed_state[idx].prev_angle;
	if (delta > ((int32_t)AS5047P_COUNT / 2))
	{
		delta -= (int32_t)AS5047P_COUNT;
	}
	else if (delta < -((int32_t)AS5047P_COUNT / 2))
	{
		delta += (int32_t)AS5047P_COUNT;
	}

	s_speed_state[idx].prev_angle = angle;

	return ((float)delta / (float)AS5047P_COUNT) * (60.0f / sample_period_s);
}

AS5047P_Data AS5047P_Read_FOC_Data(uint8_t coder_num, uint8_t pole_pairs, float sample_period_s)
{
	AS5047P_Data data;
	uint16_t raw_angle_frame;
	uint16_t raw_unc_frame;
	uint16_t raw_err_frame;
	uint16_t raw_diaagc_frame;
	uint16_t raw_mag_frame;
	float elec_pu;

	memset(&data, 0, sizeof(data));

	raw_angle_frame = AS5047P_Read_Register(AS5047P_ANGLECOM_REG, coder_num);
	raw_unc_frame = AS5047P_Read_Register(AS5047P_ANGLEUNC_REG, coder_num);
	raw_err_frame = AS5047P_Read_Register(AS5047P_ERRFL_REG, coder_num);
	raw_diaagc_frame = AS5047P_Read_Register(AS5047P_DIAAGC_REG, coder_num);
	raw_mag_frame = AS5047P_Read_Register(AS5047P_MAG_REG, coder_num);

	data.raw_angle = AS5047P_GetData(raw_angle_frame);
	data.raw_unc_angle = AS5047P_GetData(raw_unc_frame);
	data.errfl = AS5047P_GetData(raw_err_frame) & 0x0007U;
	data.diaagc = AS5047P_GetData(raw_diaagc_frame);
	data.agc_value = (uint16_t)(data.diaagc & 0x00FFU);
	data.mag_value = AS5047P_GetData(raw_mag_frame);

	data.angle_pu = (float)data.raw_angle / (float)AS5047P_COUNT;
	data.angle_deg = data.angle_pu * 360.0f;
	elec_pu = data.angle_pu * (float)pole_pairs;
	data.electrical_pu = elec_pu - floorf(elec_pu);
	data.speed_rpm = AS5047P_CalcSpeedRpm(coder_num, data.raw_angle, sample_period_s);

	data.mag_too_low = ((data.diaagc & AS5047P_DIAAGC_FIELD_LOW) != 0U) ? 1U : 0U;
	data.mag_too_high = ((data.diaagc & AS5047P_DIAAGC_FIELD_HIGH) != 0U) ? 1U : 0U;
	data.cordic_overflow = ((data.diaagc & AS5047P_DIAAGC_COF) != 0U) ? 1U : 0U;
	data.offset_comp_finished = ((data.diaagc & AS5047P_DIAAGC_LF) != 0U) ? 1U : 0U;

	data.error_flag = AS5047P_FrameHasError(raw_angle_frame);
	data.error_flag |= AS5047P_FrameHasError(raw_unc_frame);
	data.error_flag |= AS5047P_FrameHasError(raw_err_frame);
	data.error_flag |= AS5047P_FrameHasError(raw_diaagc_frame);
	data.error_flag |= AS5047P_FrameHasError(raw_mag_frame);
	data.error_flag |= (data.errfl != 0U) ? 1U : 0U;
	data.error_flag |= data.mag_too_low;
	data.error_flag |= data.mag_too_high;
	data.error_flag |= data.cordic_overflow;

	return data;
}

AS5047P_Data Read_AS5047P_Angle(uint8_t coder_num)
{
	return AS5047P_Read_FOC_Data(coder_num, POLE_PAIR_NUM, AS5047P_DEFAULT_DT_S);
}

uint8_t AS5047P_Has_Error(const AS5047P_Data *data)
{
	if (data == NULL)
	{
		return 1U;
	}

	return data->error_flag;
}

void AS5047P_Check_Encoder_Errors(AS5047P_Data *data)
{
	if ((data == NULL) || (data->error_flag == 0U))
	{
		return;
	}

	(void)AS5047P_Read_Register(AS5047P_ERRFL_REG, 1U);
	(void)AS5047P_Read_Register(AS5047P_ERRFL_REG, 2U);
}
