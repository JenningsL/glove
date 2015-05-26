#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "stm32l1xx.h"

/* Private define ------------------------------------------------------------*/
#define u8                                        uint8_t
#define u16                                       uint16_t
#define u32                                       uint32_t

#define SPI_FLASH_PageSize                        256
#define SPI_FLASH_PerWritePageSize                256
#define SPI_FLASH_SectorSize                      4096
#define SPI_FLASH_SectorCounts                    4096
#define SPI_FLASH_BlockSize                      65536

/* Flash ЦёБо */
#define W25X_WriteEnable		          0x06 
#define W25X_WriteDisable		          0x04 
#define W25X_ReadStatusReg		          0x05 
#define W25X_WriteStatusReg		          0x01 
#define W25X_ReadData			0x03 
#define W25X_FastReadData		          0x0B 
#define W25X_FastReadDual		          0x3B 
#define W25X_PageProgram		          0x02 
#define W25X_BlockErase		          0xD8 
#define W25X_SectorErase		          0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	                    0xAB 
#define W25X_DeviceID			0xAB 
#define W25X_ManufactDeviceID   	          0x90 
#define W25X_JedecDeviceID		          0x9F 

#define WIP_Flag                                  0x01  /* Write In Progress (WIP) flag */
#define Dummy_Byte                                0xFF

/* FLASH SPI Interface pins  */  
#define FLASH_SPI                                 SPI2
#define FLASH_SPI_CLK                             RCC_APB1Periph_SPI2
#define FLASH_SPI_CLK_INIT                        RCC_APB1PeriphClockCmd 

#define FLASH_SPI_SCK_PIN                         GPIO_Pin_13
#define FLASH_SPI_SCK_GPIO_PORT                   GPIOB
#define FLASH_SPI_SCK_GPIO_CLK                    RCC_AHBPeriph_GPIOB
#define FLASH_SPI_SCK_SOURCE                      GPIO_PinSource13
#define FLASH_SPI_SCK_AF                          GPIO_AF_SPI2

#define FLASH_SPI_MISO_PIN                        GPIO_Pin_14
#define FLASH_SPI_MISO_GPIO_PORT                  GPIOB
#define FLASH_SPI_MISO_GPIO_CLK                   RCC_AHBPeriph_GPIOB
#define FLASH_SPI_MISO_SOURCE                     GPIO_PinSource14
#define FLASH_SPI_MISO_AF                         GPIO_AF_SPI2

#define FLASH_SPI_MOSI_PIN                        GPIO_Pin_15
#define FLASH_SPI_MOSI_GPIO_PORT                  GPIOB
#define FLASH_SPI_MOSI_GPIO_CLK                   RCC_AHBPeriph_GPIOB
#define FLASH_SPI_MOSI_SOURCE                     GPIO_PinSource15
#define FLASH_SPI_MOSI_AF                         GPIO_AF_SPI2

#define FLASH_CS_PIN                              GPIO_Pin_12
#define FLASH_CS_GPIO_PORT                        GPIOB
#define FLASH_CS_GPIO_CLK                         RCC_AHBPeriph_GPIOB

#define ON          1
#define OFF         0
#define USE_SETOR_ERASE                           ON
#define USE_CHIP_ERASE                            ON
#define USE_PAGE_WRITE                            ON
#define USE_BUFFER_WRITE                          ON
#define USE_BUFFER_READ                           ON
#define USE_READID                                ON
#define USE_READ_DEVICEID                         ON
#define USE_STAET_READSEQUENCE                    OFF
#define USE_POWER_DOWM                            ON
#define USE_POWER_WAKEUP                          ON
#define USE_SEND_HALFWORD                         OFF

/* Exported macro ------------------------------------------------------------*/
/* Select FLASH: Chip Select pin low */
#define SPI_FLASH_CS_LOW()       FLASH_CS_GPIO_PORT->BSRRH = FLASH_CS_PIN //GPIO_ResetBits(FLASH_CS_GPIO_PORT, FLASH_CS_PIN)
/* Deselect FLASH: Chip Select pin high */
#define SPI_FLASH_CS_HIGH()      FLASH_CS_GPIO_PORT->BSRRL = FLASH_CS_PIN //GPIO_SetBits(FLASH_CS_GPIO_PORT, FLASH_CS_PIN)  

void SPI_FLASH_Init(void);
void SPI_FLASH_SectorErase(u32 SectorAddr);
void SPI_FLASH_ChipErase(void);
void SPI_FLASH_PageWrite(const u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferWrite(const u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
void SPI_FLASH_BufferRead(u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
u32 SPI_FLASH_ReadID(void);
u32 SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(u32 ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);
void spi_read_floatArr(float * rxBuffer,u32 ReadAddr, u16 NumToRead);
void spi_write_floatArr(float * txBuffer,u32 WriteAddr, u16 NumToWrite);
void spi_read_intArr(int * rxBuffer,u32 ReadAddr, u16 NumToRead);
void spi_write_intArr(int * txBuffer,u32 WriteAddr, u16 NumToWrite);

u8 SPI_FLASH_ReadByte(void);
u8 SPI_FLASH_SendByte(u8 byte);
u16 SPI_FLASH_SendHalfWord(u16 HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);
#endif /* __SPI_FLASH_H */

