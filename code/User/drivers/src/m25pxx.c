/**
 ******************************************************************************
 * @file    SPI/SPI_FLASH/spi_flash.c
 * @author  MCD Application Team
 * @version V1.3.0
 * @date    13-November-2013
 * @brief   This file provides a set of functions needed to manage the SPI M25Pxxx
 *          FLASH memory.
 *
 *          ===================================================================
 *          Notes:
 *           - There is no SPI FLASH memory available in STM324xG-EVAL board,
 *             to use this driver you have to build your own hardware.
 *          ===================================================================
 *
 *          It implements a high level communication layer for read and write
 *          from/to this memory. The needed STM32 hardware resources (SPI and
 *          GPIO) are defined in spi_flash.h file, and the initialization is
 *          performed in M25PXX_LowLevel_Init() function.
 *
 *          You can easily tailor this driver to any development board, by just
 *          adapting the defines for hardware resources and M25PXX_LowLevel_Init()
 *          function.
 *
 *          +-----------------------------------------------------------+
 *          |                     Pin assignment                        |
 *          +-----------------------------+---------------+-------------+
 *          |  STM32 SPI Pins             |     M25PXX    |    Pin      |
 *          +-----------------------------+---------------+-------------+
 *          | M25PXX_CS_PIN               | ChipSelect(/S)|    1        |
 *          | M25PXX_SPI_MISO_PIN / MISO  |   DataOut(Q)  |    2        |
 *          |                             |   VCC         |    3 (3.3 V)|
 *          |                             |   GND         |    4 (0 V)  |
 *          | M25PXX_SPI_MOSI_PIN / MOSI  |   DataIn(D)   |    5        |
 *          | M25PXX_SPI_SCK_PIN / SCK    |   Clock(C)    |    6        |
 *          |                             |    VCC        |    7 (3.3 V)|
 *          |                             |    VCC        |    8 (3.3 V)|
 *          +-----------------------------+---------------+-------------+
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "m25pxx.h"
#include "stm32f10x_spi.h"
#include "drivers.h"

/** @addtogroup STM32F4xx_StdPeriph_Examples
 * @{
 */

/** @addtogroup SPI_FLASH
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void M25PXX_LowLevel_DeInit(void);
void M25PXX_LowLevel_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 * @param  None
 * @retval None
 */
void M25PXX_DeInit(void)
{
    M25PXX_LowLevel_DeInit();
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @param  None
 * @retval None
 */
void M25PXX_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;

    M25PXX_LowLevel_Init();

    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();

    /*!< SPI configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(M25PXX_SPI, &SPI_InitStructure);

    /*!< Enable the M25PXX_SPI  */
    SPI_Cmd(M25PXX_SPI, ENABLE);
}

/**
 * @brief  Erases the specified FLASH sector.
 * @param  SectorAddr: address of the sector to erase.
 * @retval None
 */
void M25PXX_EraseSector(uint32_t SectorAddr)
{
    /*!< Send write enable instruction */
    M25PXX_WriteEnable();

    /*!< Sector Erase */
    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();
    /*!< Send Sector Erase instruction */
    M25PXX_SendByte(M25PXX_CMD_SE);
    /*!< Send SectorAddr high nibble address byte */
    M25PXX_SendByte((SectorAddr & 0xFF0000) >> 16);
    /*!< Send SectorAddr medium nibble address byte */
    M25PXX_SendByte((SectorAddr & 0xFF00) >> 8);
    /*!< Send SectorAddr low nibble address byte */
    M25PXX_SendByte(SectorAddr & 0xFF);
    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();

    /*!< Wait the end of Flash writing */
    M25PXX_WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 * @retval None
 */
void M25PXX_EraseBulk(void)
{
    /*!< Send write enable instruction */
    M25PXX_WriteEnable();

    /*!< Bulk Erase */
    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();
    /*!< Send Bulk Erase instruction  */
    M25PXX_SendByte(M25PXX_CMD_BE);
    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();

    /*!< Wait the end of Flash writing */
    M25PXX_WaitForWriteEnd();
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "M25PXX_PAGESIZE" value.
 * @retval None
 */
void M25PXX_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    /*!< Enable the write access to the FLASH */
    M25PXX_WriteEnable();

    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();
    /*!< Send "Write to Memory " instruction */
    M25PXX_SendByte(M25PXX_CMD_WRITE);
    /*!< Send WriteAddr high nibble address byte to write to */
    M25PXX_SendByte((WriteAddr & 0xFF0000) >> 16);
    /*!< Send WriteAddr medium nibble address byte to write to */
    M25PXX_SendByte((WriteAddr & 0xFF00) >> 8);
    /*!< Send WriteAddr low nibble address byte to write to */
    M25PXX_SendByte(WriteAddr & 0xFF);

    /*!< while there is data to be written on the FLASH */
    while (NumByteToWrite--)
    {
        /*!< Send the current byte */
        M25PXX_SendByte(*pBuffer);
        /*!< Point on the next byte to be written */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();

    /*!< Wait the end of Flash writing */
    M25PXX_WaitForWriteEnd();
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 * @retval None
 */
void M25PXX_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = WriteAddr % M25PXX_SPI_PAGESIZE;
    count = M25PXX_SPI_PAGESIZE - Addr;
    NumOfPage =  NumByteToWrite / M25PXX_SPI_PAGESIZE;
    NumOfSingle = NumByteToWrite % M25PXX_SPI_PAGESIZE;

    if (Addr == 0) /*!< WriteAddr is M25PXX_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < M25PXX_PAGESIZE */
        {
            M25PXX_WritePage(pBuffer, WriteAddr, NumByteToWrite);
        }
        else /*!< NumByteToWrite > M25PXX_PAGESIZE */
        {
            while (NumOfPage--)
            {
                M25PXX_WritePage(pBuffer, WriteAddr, M25PXX_SPI_PAGESIZE);
                WriteAddr +=  M25PXX_SPI_PAGESIZE;
                pBuffer += M25PXX_SPI_PAGESIZE;
            }

            M25PXX_WritePage(pBuffer, WriteAddr, NumOfSingle);
        }
    }
    else /*!< WriteAddr is not M25PXX_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < M25PXX_PAGESIZE */
        {
            if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > M25PXX_PAGESIZE */
            {
                temp = NumOfSingle - count;

                M25PXX_WritePage(pBuffer, WriteAddr, count);
                WriteAddr +=  count;
                pBuffer += count;

                M25PXX_WritePage(pBuffer, WriteAddr, temp);
            }
            else
            {
                M25PXX_WritePage(pBuffer, WriteAddr, NumByteToWrite);
            }
        }
        else /*!< NumByteToWrite > M25PXX_PAGESIZE */
        {
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / M25PXX_SPI_PAGESIZE;
            NumOfSingle = NumByteToWrite % M25PXX_SPI_PAGESIZE;

            M25PXX_WritePage(pBuffer, WriteAddr, count);
            WriteAddr +=  count;
            pBuffer += count;

            while (NumOfPage--)
            {
                M25PXX_WritePage(pBuffer, WriteAddr, M25PXX_SPI_PAGESIZE);
                WriteAddr +=  M25PXX_SPI_PAGESIZE;
                pBuffer += M25PXX_SPI_PAGESIZE;
            }

            if (NumOfSingle != 0)
            {
                M25PXX_WritePage(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 * @retval None
 */
void M25PXX_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();

    /*!< Send "Read from Memory " instruction */
    M25PXX_SendByte(M25PXX_CMD_READ);

    /*!< Send ReadAddr high nibble address byte to read from */
    M25PXX_SendByte((ReadAddr & 0xFF0000) >> 16);
    /*!< Send ReadAddr medium nibble address byte to read from */
    M25PXX_SendByte((ReadAddr& 0xFF00) >> 8);
    /*!< Send ReadAddr low nibble address byte to read from */
    M25PXX_SendByte(ReadAddr & 0xFF);

    while (NumByteToRead--) /*!< while there is data to be read */
    {
        /*!< Read a byte from the FLASH */
        *pBuffer = M25PXX_SendByte(M25PXX_DUMMY_BYTE);
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();
}

/**
 * @brief  Reads FLASH identification.
 * @param  None
 * @retval FLASH identification
 */
uint32_t M25PXX_ReadID(void)
{
    uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();

    /*!< Send "RDID " instruction */
    M25PXX_SendByte(0x9F);

    /*!< Read a byte from the FLASH */
    Temp0 = M25PXX_SendByte(M25PXX_DUMMY_BYTE);

    /*!< Read a byte from the FLASH */
    Temp1 = M25PXX_SendByte(M25PXX_DUMMY_BYTE);

    /*!< Read a byte from the FLASH */
    Temp2 = M25PXX_SendByte(M25PXX_DUMMY_BYTE);

    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();

    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return Temp;
}

/**
 * @brief  Initiates a read data byte (READ) sequence from the Flash.
 *   This is done by driving the /CS line low to select the device, then the READ
 *   instruction is transmitted followed by 3 bytes address. This function exit
 *   and keep the /CS line low, so the Flash still being selected. With this
 *   technique the whole content of the Flash is read with a single READ instruction.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @retval None
 */
void M25PXX_StartReadSequence(uint32_t ReadAddr)
{
    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();

    /*!< Send "Read from Memory " instruction */
    M25PXX_SendByte(M25PXX_CMD_READ);

    /*!< Send the 24-bit address of the address to read from -------------------*/
    /*!< Send ReadAddr high nibble address byte */
    M25PXX_SendByte((ReadAddr & 0xFF0000) >> 16);
    /*!< Send ReadAddr medium nibble address byte */
    M25PXX_SendByte((ReadAddr& 0xFF00) >> 8);
    /*!< Send ReadAddr low nibble address byte */
    M25PXX_SendByte(ReadAddr & 0xFF);
}

/**
 * @brief  Reads a byte from the SPI Flash.
 * @note   This function must be used only if the Start_Read_Sequence function
 *         has been previously called.
 * @param  None
 * @retval Byte Read from the SPI Flash.
 */
uint8_t M25PXX_ReadByte(void)
{
    return (M25PXX_SendByte(M25PXX_DUMMY_BYTE));
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @retval The value of the received byte.
 */
uint8_t M25PXX_SendByte(uint8_t byte)
{
    /*!< Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(M25PXX_SPI, SPI_I2S_FLAG_TXE) == RESET);

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(M25PXX_SPI, byte);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(M25PXX_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(M25PXX_SPI);
}

/**
 * @brief  Sends a Half Word through the SPI interface and return the Half Word
 *         received from the SPI bus.
 * @param  HalfWord: Half Word to send.
 * @retval The value of the received Half Word.
 */
uint16_t M25PXX_SendHalfWord(uint16_t HalfWord)
{
    /*!< Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(M25PXX_SPI, SPI_I2S_FLAG_TXE) == RESET);

    /*!< Send Half Word through the M25PXX peripheral */
    SPI_I2S_SendData(M25PXX_SPI, HalfWord);

    /*!< Wait to receive a Half Word */
    while (SPI_I2S_GetFlagStatus(M25PXX_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    /*!< Return the Half Word read from the SPI bus */
    return SPI_I2S_ReceiveData(M25PXX_SPI);
}

/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @retval None
 */
void M25PXX_WriteEnable(void)
{
    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();

    /*!< Send "Write Enable" instruction */
    M25PXX_SendByte(M25PXX_CMD_WREN);

    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @param  None
 * @retval None
 */
void M25PXX_WaitForWriteEnd(void)
{
    uint8_t flashstatus = 0;

    /*!< Select the FLASH: Chip Select low */
    M25PXX_CS_LOW();

    /*!< Send "Read Status Register" instruction */
    M25PXX_SendByte(M25PXX_CMD_RDSR);

    /*!< Loop as long as the memory is busy with a write cycle */
    do
    {
        /*!< Send a dummy byte to generate the clock needed by the FLASH
          and put the value of the status register in FLASH_Status variable */
        flashstatus = M25PXX_SendByte(M25PXX_DUMMY_BYTE);

    }
    while ((flashstatus & M25PXX_WIP_FLAG) == SET); /* Write in progress */

    /*!< Deselect the FLASH: Chip Select high */
    M25PXX_CS_HIGH();
}

/**
 * @brief  Initializes the peripherals used by the SPI FLASH driver.
 * @param  None
 * @retval None
 */
void M25PXX_LowLevel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*!< Enable the SPI clock */
    M25PXX_SPI_CLK_INIT(M25PXX_SPI_CLK, ENABLE);

    /*!< Enable GPIO clocks */
    RCC_APB2PeriphClockCmd(M25PXX_SPI_SCK_GPIO_CLK | M25PXX_SPI_MISO_GPIO_CLK |
            M25PXX_SPI_MOSI_GPIO_CLK | M25PXX_CS_GPIO_CLK, ENABLE);

    /*!< SPI pins configuration *************************************************/

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /*!< SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = M25PXX_SPI_SCK_PIN;
    GPIO_Init(M25PXX_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /*!< SPI MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  M25PXX_SPI_MOSI_PIN;
    GPIO_Init(M25PXX_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /*!< SPI MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin =  M25PXX_SPI_MISO_PIN;
    GPIO_Init(M25PXX_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure M25PXX Card CS pin in output pushpull mode ********************/
    GPIO_InitStructure.GPIO_Pin = M25PXX_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(M25PXX_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  DeInitializes the peripherals used by the SPI FLASH driver.
 * @param  None
 * @retval None
 */
void M25PXX_LowLevel_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*!< Disable the M25PXX_SPI  ************************************************/
    SPI_Cmd(M25PXX_SPI, DISABLE);

    /*!< DeInitializes the M25PXX_SPI *******************************************/
    SPI_I2S_DeInit(M25PXX_SPI);

    /*!< M25PXX_SPI Periph clock disable ****************************************/
    M25PXX_SPI_CLK_INIT(M25PXX_SPI_CLK, DISABLE);

    /*!< Configure all pins used by the SPI as input floating *******************/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

    GPIO_InitStructure.GPIO_Pin = M25PXX_SPI_SCK_PIN;
    GPIO_Init(M25PXX_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = M25PXX_SPI_MISO_PIN;
    GPIO_Init(M25PXX_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = M25PXX_SPI_MOSI_PIN;
    GPIO_Init(M25PXX_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = M25PXX_CS_PIN;
    GPIO_Init(M25PXX_CS_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
