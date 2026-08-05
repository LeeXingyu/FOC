/*
 * define.h
 *
 *  Created on: Jun 1, 2026
 *      Author: Administrator
 */

#ifndef INC_BOARD_LAN9252_DEFINE_H_
#define INC_BOARD_LAN9252_DEFINE_H_

#define RST_ESC()     	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
#define RST_ESC_END()  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

#define FLASH_SPIx                                 SPI2

#define FLASH_SPI_CS_ClK_ENABLE()                  __HAL_RCC_GPIOA_CLK_ENABLE()
#define FLASH_SPI_CS_PORT                          GPIOB
#define FLASH_SPI_CS_PIN                           GPIO_PIN_12

#define DESELECT_SPI                          HAL_GPIO_WritePin(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, GPIO_PIN_SET)
#define SELECT_SPI                            HAL_GPIO_WritePin(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, GPIO_PIN_RESET)

#define LAN9253_CS_LOW()      SELECT_SPI
#define LAN9253_CS_HIGH()     DESELECT_SPI

//static inline uint8_t SPIRW(uint8_t tx)
//{
//    uint8_t rx;
//    HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1, 100);
//    return rx;
//}
//#define SPIWriteByte(b)  (void)SPIRW(b)
//#define SPIReadByte()    SPIRW(0xFF)


#define SPIWriteByte SPIWrite
#define SPIReadByte() SPIRead()

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Data Types
// *****************************************************************************
// *****************************************************************************
#define CMD_SERIAL_READ 0x03
#define CMD_FAST_READ 0x0B
#define CMD_DUAL_OP_READ 0x3B
#define CMD_DUAL_IO_READ 0xBB
#define CMD_QUAD_OP_READ 0x6B
#define CMD_QUAD_IO_READ 0xEB
#define CMD_SERIAL_WRITE 0x02
#define CMD_DUAL_DATA_WRITE 0x32
#define CMD_DUAL_ADDR_DATA_WRITE 0xB2
#define CMD_QUAD_DATA_WRITE 0x62
#define CMD_QUAD_ADDR_DARA_WRITE 0xE2

#define CMD_SERIAL_READ_DUMMY 0
#define CMD_FAST_READ_DUMMY 1
#define CMD_DUAL_OP_READ_DUMMY 1
#define CMD_DUAL_IO_READ_DUMMY 2
#define CMD_QUAD_OP_READ_DUMMY 1
#define CMD_QUAD_IO_READ_DUMMY 4
#define CMD_SERIAL_WRITE_DUMMY 0
#define CMD_DUAL_DATA_WRITE_DUMMY 0
#define CMD_DUAL_ADDR_DATA_WRITE_DUMMY 0
#define CMD_QUAD_DATA_WRITE_DUMMY 0
#define CMD_QUAD_ADDR_DARA_WRITE_DUMMY 0

#define ESC_CSR_CMD_REG		0x304
#define ESC_CSR_DATA_REG	0x300
#define ESC_WRITE_BYTE 		0x80
#define ESC_READ_BYTE 		0xC0
#define ESC_CSR_BUSY		0x80

//9252 HW DEFINES
#define ECAT_REG_BASE_ADDR              0x0300

#define CSR_DATA_REG_OFFSET             0x00
#define CSR_CMD_REG_OFFSET              0x04
#define PRAM_READ_ADDR_LEN_OFFSET       0x08
#define PRAM_READ_CMD_OFFSET            0x0c
#define PRAM_WRITE_ADDR_LEN_OFFSET      0x10
#define PRAM_WRITE_CMD_OFFSET           0x14

#define PRAM_SPACE_AVBL_COUNT_MASK      0x1f
#define IS_PRAM_SPACE_AVBL_MASK         0x01

#define PRAM_RW_ABORT_MASK      ((unsigned long)1 << 30)
#define PRAM_RW_BUSY_32B        ((unsigned long)1 << 31)
#define PRAM_RW_BUSY_8B         ((unsigned long)1 << 7)
#define PRAM_SET_READ           ((unsigned long)1 << 6)
#define PRAM_SET_WRITE          0

#define CSR_DATA_REG                    ECAT_REG_BASE_ADDR+CSR_DATA_REG_OFFSET
#define CSR_CMD_REG                     ECAT_REG_BASE_ADDR+CSR_CMD_REG_OFFSET
#define PRAM_READ_ADDR_LEN_REG          ECAT_REG_BASE_ADDR+PRAM_READ_ADDR_LEN_OFFSET
#define PRAM_READ_CMD_REG               ECAT_REG_BASE_ADDR+PRAM_READ_CMD_OFFSET
#define PRAM_WRITE_ADDR_LEN_REG         ECAT_REG_BASE_ADDR+PRAM_WRITE_ADDR_LEN_OFFSET
#define PRAM_WRITE_CMD_REG              ECAT_REG_BASE_ADDR+PRAM_WRITE_CMD_OFFSET

#define PRAM_READ_FIFO_REG              0x04
#define PRAM_WRITE_FIFO_REG             0x20

//#define ECAT_RST_Pin       GPIO_PIN_9
//#define ECAT_RST_GPIO_Port GPIOC

#define ECAT_INT_Pin       GPIO_PIN_10
#define ECAT_INT_GPIO_Port GPIOC
#define ECAT_INT_EXTI_IRQn EXTI15_10_IRQn

#define SYNC0_Pin          GPIO_PIN_15
#define SYNC0_GPIO_Port    GPIOA
#define SYNC0_EXTI_IRQn    EXTI15_10_IRQn

#define SYNC1_Pin          GPIO_PIN_0
#define SYNC1_GPIO_Port    GPIOC
#define SYNC1_EXTI_IRQn    EXTI0_IRQn



#endif /* INC_BOARD_LAN9252_DEFINE_H_ */
