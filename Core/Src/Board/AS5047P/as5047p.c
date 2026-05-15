/*
 * as5047p.c
 *
 *  Created on: Apr 13, 2026
 *      Author: Administrator
 */
#include "as5047p.h"


void AS5047P_Encoder_CS(uint8_t coder_num, uint8_t status)
{
	uint8_t data = coder_num < 1 | status;
	switch(data)
	{
		case 4:CODER_CS_LOW_1();break;
		case 5:CODER_CS_HIGH_1(); break;
		case 6:CODER_CS_LOW_2();break;
		case 7:CODER_CS_HIGH_2();break;
		default:CODER_CS_HIGH_2();CODER_CS_HIGH_1();break;
	}

}

// SPI传输函数 (16位数据)
static uint16_t AS5047P_SPI_Transfer(uint16_t data) {
  uint16_t rx_data;
  HAL_SPI_TransmitReceive(&hspi3, &data, &rx_data, 1, HAL_MAX_DELAY);
  return rx_data;
}

// 读取AS5047P寄存器
static uint16_t AS5047P_Read_Register(uint16_t reg_addr,uint8_t coder_num) {
  uint16_t cmd = (reg_addr & 0x3FFF) | 0x4000;  // 读命令: 0b01<13位地址>
  AS5047P_Encoder_CS(coder_num, 0);
  uint16_t result = AS5047P_SPI_Transfer(cmd);
  AS5047P_Encoder_CS(coder_num, 1);
  return result;
}

// 读取角度和状态信息
AS5047P_Data Read_AS5047P_Angle(uint8_t coder_num) {
  AS5047P_Data data = {0};
  uint16_t status_reg;

  // 1. 读取角度值 (使用简化方法: 发送0xFFFF)
  AS5047P_Encoder_CS(coder_num, 0);
  uint16_t raw_data = AS5047P_SPI_Transfer(AS5047P_READ_CMD);
  AS5047P_Encoder_CS(coder_num, 1);

  // 2. 解析数据 (高14位为角度值，低2位为状态)
  data.raw_angle = (raw_data >> 2) & 0x3FFF;  // 提取14位角度值
  data.angle_deg = (data.raw_angle * 360.0f) / 16384.0f;  // 转换为角度

  // 3. 读取状态寄存器
  status_reg = AS5047P_Read_Register(AS5047P_ANGLE_REG,coder_num);
  data.error_flag = (status_reg & 0x0002) ? 1 : 0;  // 错误标志位 (bit1)

  // 4. 读取AGC和磁场强度 (可选)
  data.agc_value = AS5047P_Read_Register(AS5047P_AGC_REG,coder_num) & 0x00FF;
  data.mag_status = AS5047P_Read_Register(AS5047P_MAG_REG,coder_num) & 0x3FFF;

  return data;
}

// 错误检测与恢复
void AS5047P_Check_Encoder_Errors(AS5047P_Data *data) {
  if(data->error_flag) {
    // 1. 清除错误标志
	AS5047P_Encoder_CS(1, 0);
	AS5047P_Encoder_CS(2, 0);
	AS5047P_SPI_Transfer(AS5047P_CLEAR_ERROR);
    AS5047P_Encoder_CS(1, 1);
    AS5047P_Encoder_CS(2, 1);
    // 2. 重新初始化SPI
    HAL_SPI_DeInit(&hspi3);
    MX_SPI3_Init();

    // 3. 记录错误日志
    Error_Handler();
  }

  // 磁场强度过低检测
  if(data->mag_status < 100) {
    // 磁铁可能松动或远离
   // Set_Error_LED(1);
  }
}
