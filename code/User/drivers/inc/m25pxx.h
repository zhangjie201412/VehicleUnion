#ifndef __M25PXX_H__
#define __M25PXX_H__

#include "stm32f10x.h"
#include "drivers.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* M25P SPI Flash supported commands */
#define M25PXX_CMD_WRITE          0x02  /* Write to Memory instruction */
#define M25PXX_CMD_WRSR           0x01  /* Write Status Register instruction */
#define M25PXX_CMD_WREN           0x06  /* Write enable instruction */
#define M25PXX_CMD_READ           0x03  /* Read from Memory instruction */
#define M25PXX_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define M25PXX_CMD_RDID           0x9F  /* Read identification */
#define M25PXX_CMD_SE             0xD8  /* Sector Erase instruction */
#define M25PXX_CMD_BE             0xC7  /* Bulk Erase instruction */

#define M25PXX_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define M25PXX_DUMMY_BYTE         0xA5
#define M25PXX_SPI_PAGESIZE       0x100

#define M25PXX_M25P128_ID         0x202018
#define M25PXX_M25P64_ID          0x202017
#define M25PXX_M25P16_ID          0x202014

/* M25P FLASH SPI Interface pins  */
#define M25PXX_SPI                           SPI2
#define M25PXX_SPI_CLK                       RCC_APB1Periph_SPI2
#define M25PXX_SPI_CLK_INIT                  RCC_APB1PeriphClockCmd

#define M25PXX_SPI_SCK_PIN                   GPIO_Pin_13
#define M25PXX_SPI_SCK_GPIO_PORT             GPIOB
#define M25PXX_SPI_SCK_GPIO_CLK              RCC_APB2Periph_GPIOB

#define M25PXX_SPI_MISO_PIN                  GPIO_Pin_14
#define M25PXX_SPI_MISO_GPIO_PORT            GPIOB
#define M25PXX_SPI_MISO_GPIO_CLK             RCC_APB2Periph_GPIOB

#define M25PXX_SPI_MOSI_PIN                  GPIO_Pin_15
#define M25PXX_SPI_MOSI_GPIO_PORT            GPIOB
#define M25PXX_SPI_MOSI_GPIO_CLK             RCC_APB2Periph_GPIOB

#define M25PXX_CS_PIN                        GPIO_Pin_12
#define M25PXX_CS_GPIO_PORT                  GPIOB
#define M25PXX_CS_GPIO_CLK                   RCC_APB2Periph_GPIOB

/* Exported macro ------------------------------------------------------------*/
/* Select M25PXX: Chip Select pin low */
#define M25PXX_CS_LOW()       GPIO_ResetBits(M25PXX_CS_GPIO_PORT, M25PXX_CS_PIN)
/* Deselect M25PXX: Chip Select pin high */
#define M25PXX_CS_HIGH()      GPIO_SetBits(M25PXX_CS_GPIO_PORT, M25PXX_CS_PIN)

/* Exported functions ------------------------------------------------------- */

/* High layer functions  */
void M25PXX_DeInit(void);
void M25PXX_Init(void);
void M25PXX_EraseSector(uint32_t SectorAddr);
void M25PXX_EraseBulk(void);
void M25PXX_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void M25PXX_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void M25PXX_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t M25PXX_ReadID(void);
void M25PXX_StartReadSequence(uint32_t ReadAddr);

/* Low layer functions */
uint8_t M25PXX_ReadByte(void);
uint8_t M25PXX_SendByte(uint8_t byte);
uint16_t M25PXX_SendHalfWord(uint16_t HalfWord);
void M25PXX_WriteEnable(void);
void M25PXX_WaitForWriteEnd(void);

#endif
