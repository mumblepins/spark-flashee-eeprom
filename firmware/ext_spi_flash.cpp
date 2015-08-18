/**
 ******************************************************************************
 * @file    ext_spi_flash.c
 * @author  Daniel Sullivan
 * @version V1.0.0
 * @date    11-Aug-2015
 * @brief   spi flash driver (modeled after spi_flash.c from particle.io)
 ******************************************************************************
  Copyright (c) 2013-2015 Particle Industries, Inc.  All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/
#include "ext_spi_flash.h"
ExtSpiFlash::ExtSpiFlash(void) {
}

ExtSpiFlash::ExtSpiFlash(uint32_t jedecID, uint16_t csPin, uint32_t pageSize, uint32_t pageCount,  uint8_t clockDivider) {
    _jedec_id=jedecID;
    _cs_pin=csPin;
    _page_size=pageSize;
    _page_count=pageCount;
    _clock_divider=clockDivider;
}

uint32_t ExtSpiFlash::pageSize(void) {
    return _page_size;
}

uint32_t ExtSpiFlash::pageCount(void) {
    return _page_count;
}

/**
 * @brief Sets Chip Select High
 * @param void
 * @return void
 */
 void ExtSpiFlash::CS_HIGH(void) {
    digitalWrite(_cs_pin, HIGH);
}

/**
 * @brief Sets Chip Select Low
 * @param void
 * @return void
 */
void ExtSpiFlash::CS_LOW(void) {
    digitalWrite(_cs_pin, LOW);
}

/**
 * @brief Initializes SPI Bus
 * @param void
 * @return void
 */
void ExtSpiFlash::SPI_Init(void) {
    pinMode(_cs_pin,OUTPUT);

    SPI.begin(_cs_pin);
    CS_HIGH();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);
    SPI.setClockDivider(_clock_divider);

}

/**
 * @brief Initializes SPI Flash
 * @param void
 * @return void
 */
void ExtSpiFlash::Init(void)
{
    /* Initializes the peripherals used by the SPI FLASH driver */
    SPI_Init();

    /* Disable the write access to the FLASH */
    WriteDisable();

    /* Select the FLASH: Chip Select low */
    CS_LOW();
    /* Send "Write Enable Status" instruction */
    SendByte(extFLASH_CMD_EWSR);
    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();

    /* Select the FLASH: Chip Select low */
    CS_LOW();
    /* Send "Write Status Register" instruction */
    SendByte(extFLASH_CMD_WRSR);
    SendByte(0);
    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();
}

/**
 * @brief  Erases the specified FLASH sector.
 * @param  SectorAddr: address of the sector to erase.
 * @retval None
 */
void ExtSpiFlash::EraseSector(uint32_t SectorAddr)
{
    /* Enable the write access to the FLASH */
    WriteEnable();

    /* Sector Erase */
    /* Select the FLASH: Chip Select low */
    CS_LOW();
    /* Send Sector Erase instruction */
    SendByte(extFLASH_CMD_SE);
    /* Send SectorAddr high nibble address byte */
    SendByte((SectorAddr & 0xFF0000) >> 16);
    /* Send SectorAddr medium nibble address byte */
    SendByte((SectorAddr & 0xFF00) >> 8);
    /* Send SectorAddr low nibble address byte */
    SendByte(SectorAddr & 0xFF);
    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();

    /* Wait till the end of Flash writing */
    WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 * @retval None
 */
void ExtSpiFlash::EraseBulk(void)
{
    /* Enable the write access to the FLASH */
    WriteEnable();

    /* Bulk Erase */
    /* Select the FLASH: Chip Select low */
    CS_LOW();
    /* Send Bulk Erase instruction  */
    SendByte(extFLASH_CMD_BE);
    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();

    /* Wait till the end of Flash writing */
    WaitForWriteEnd();
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "extFLASH_PROGRAM_PAGESIZE" value.
 * @retval None
 */
void ExtSpiFlash::WritePage(const uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
    /* Enable the write access to the FLASH */
    WriteEnable();

    /* Select the FLASH: Chip Select low */
    CS_LOW();
    /* Send "Write to Memory " instruction */
    SendByte(extFLASH_CMD_WRITE);
    /* Send WriteAddr high nibble address byte to write to */
    SendByte((WriteAddr & 0xFF0000) >> 16);
    /* Send WriteAddr medium nibble address byte to write to */
    SendByte((WriteAddr & 0xFF00) >> 8);
    /* Send WriteAddr low nibble address byte to write to */
    SendByte(WriteAddr & 0xFF);

    /* while there is data to be written on the FLASH */
    while (NumByteToWrite)
    {
        /* Send the current byte and point to the next location */
        SendByte(*pBuffer++);
        /* Decrement NumByteToWrite */
        NumByteToWrite--;
    }

    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();

    /* Wait the end of Flash writing */
    WaitForWriteEnd();
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @note   Addresses to be written must be in the erased state
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 * @retval None
 */
void ExtSpiFlash::WriteBuffer(const uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = WriteAddr % extFLASH_PROGRAM_PAGESIZE;
    count = extFLASH_PROGRAM_PAGESIZE - Addr;
    NumOfPage =  NumByteToWrite / extFLASH_PROGRAM_PAGESIZE;
    NumOfSingle = NumByteToWrite % extFLASH_PROGRAM_PAGESIZE;

    if (Addr == 0) /* WriteAddr is extFLASH_PROGRAM_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /* NumByteToWrite < extFLASH_PROGRAM_PAGESIZE */
        {
            WritePage(pBuffer, WriteAddr, NumByteToWrite);
        }
        else /* NumByteToWrite > extFLASH_PROGRAM_PAGESIZE */
        {
            while (NumOfPage--)
            {
                WritePage(pBuffer, WriteAddr, extFLASH_PROGRAM_PAGESIZE);
                WriteAddr +=  extFLASH_PROGRAM_PAGESIZE;
                pBuffer += extFLASH_PROGRAM_PAGESIZE;
            }

            WritePage(pBuffer, WriteAddr, NumOfSingle);
        }
    }
    else /* WriteAddr is not extFLASH_PROGRAM_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /* NumByteToWrite < extFLASH_PROGRAM_PAGESIZE */
        {
            if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > extFLASH_PROGRAM_PAGESIZE */
            {
                temp = NumOfSingle - count;

                WritePage(pBuffer, WriteAddr, count);
                WriteAddr +=  count;
                pBuffer += count;

                WritePage(pBuffer, WriteAddr, temp);
            }
            else
            {
                WritePage(pBuffer, WriteAddr, NumByteToWrite);
            }
        }
        else /* NumByteToWrite > extFLASH_PROGRAM_PAGESIZE */
        {
            NumByteToWrite -= count;
            NumOfPage =  NumByteToWrite / extFLASH_PROGRAM_PAGESIZE;
            NumOfSingle = NumByteToWrite % extFLASH_PROGRAM_PAGESIZE;

            WritePage(pBuffer, WriteAddr, count);
            WriteAddr +=  count;
            pBuffer += count;

            while (NumOfPage--)
            {
                WritePage(pBuffer, WriteAddr, extFLASH_PROGRAM_PAGESIZE);
                WriteAddr +=  extFLASH_PROGRAM_PAGESIZE;
                pBuffer += extFLASH_PROGRAM_PAGESIZE;
            }

            if (NumOfSingle != 0)
            {
                WritePage(pBuffer, WriteAddr, NumOfSingle);
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
void ExtSpiFlash::ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    /* Select the FLASH: Chip Select low */
    CS_LOW();

    /* Send "Read from Memory " instruction */
    SendByte(extFLASH_CMD_READ);

    /* Send ReadAddr high nibble address byte to read from */
    SendByte((ReadAddr & 0xFF0000) >> 16);
    /* Send ReadAddr medium nibble address byte to read from */
    SendByte((ReadAddr& 0xFF00) >> 8);
    /* Send ReadAddr low nibble address byte to read from */
    SendByte(ReadAddr & 0xFF);

    while (NumByteToRead) /* while there is data to be read */
    {
        /* Read a byte from the FLASH and point to the next location */
        *pBuffer++ = SendByte(extFLASH_DUMMY_BYTE);
        /* Decrement NumByteToRead */
        NumByteToRead--;
    }

    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();
}

/**
 * @brief  Reads FLASH identification.
 * @param  None
 * @retval FLASH identification
 */
uint32_t ExtSpiFlash::ReadID(void)
{
    uint8_t byte[3];

    /* Select the FLASH: Chip Select low */
    CS_LOW();

    /* Send "JEDEC ID Read" instruction */
    SendByte(extFLASH_CMD_RDID);

    /* Read a byte from the FLASH */
    byte[0] = SendByte(extFLASH_DUMMY_BYTE);

    /* Read a byte from the FLASH */
    byte[1] = SendByte(extFLASH_DUMMY_BYTE);

    /* Read a byte from the FLASH */
    byte[2] = SendByte(extFLASH_DUMMY_BYTE);

    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();

    return (byte[0] << 16) | (byte[1] << 8) | byte[2];
}

/**
 * @brief  Sends a byte through the SPI interface and return the byte received
 *         from the SPI bus.
 * @param  byte: byte to send.
 * @retval The value of the received byte.
 */
uint8_t ExtSpiFlash::SendByte(uint8_t byte)
{
    /* Loop while DR register in not empty */
    //while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_TXE) == RESET);

    /* Send byte through the SPI peripheral */
    //SPI_I2S_SendData(sFLASH_SPI, byte);

    /* Wait to receive a byte */
    //while (SPI_I2S_GetFlagStatus(sFLASH_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    /* Return the byte read from the SPI bus */
    //return SPI_I2S_ReceiveData(sFLASH_SPI);
    return SPI.transfer(byte);
}

/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @retval None
 */
void ExtSpiFlash::WriteEnable(void)
{
    /* Select the FLASH: Chip Select low */
    CS_LOW();

    /* Send "Write Enable" instruction */
    SendByte(extFLASH_CMD_WREN);

    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();
}

/**
 * @brief  Disables the write access to the FLASH.
 * @param  None
 * @retval None
 */
void ExtSpiFlash::WriteDisable(void)
{
    /* Select the FLASH: Chip Select low */
    CS_LOW();

    /* Send "Write Disable" instruction */
    SendByte(extFLASH_CMD_WRDI);

    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write operation has completed.
 * @param  None
 * @retval None
 */
void ExtSpiFlash::WaitForWriteEnd(void)
{
    uint8_t flashstatus = 0;

    /* Select the FLASH: Chip Select low */
    CS_LOW();

    /* Send "Read Status Register" instruction */
    SendByte(extFLASH_CMD_RDSR);

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
        flashstatus = SendByte(extFLASH_DUMMY_BYTE);
    }
    while ((flashstatus & extFLASH_WIP_FLAG) == SET); /* Write in progress */

    /* Deselect the FLASH: Chip Select high */
    CS_HIGH();
}

int ExtSpiFlash::SelfTest(void)
{
    uint32_t FLASH_TestAddress = 0x000000;
    //Note: Make sure BufferSize should be Even and not Zero
    uint8_t Tx_Buffer[] = "Test communication with SPI FLASH!";//BufferSize = 34
    uint32_t BufferSize = (sizeof(Tx_Buffer) / sizeof(*(Tx_Buffer))) - 1;
    uint8_t Rx_Buffer[BufferSize];
    uint8_t Index = 0;
    uint32_t FlashID = 0;
    int TestStatus = -1;

    /* Get SPI Flash ID */
    FlashID = ReadID();
    /* Check the SPI Flash ID */
    if(FlashID == _jedec_id)
    {
        /* Perform a write in the Flash followed by a read of the written data */
        /* Erase SPI FLASH Sector to write on */
        EraseSector(FLASH_TestAddress);

        /* Write Tx_Buffer data to SPI FLASH memory */
        WriteBuffer(Tx_Buffer, FLASH_TestAddress, BufferSize);

        /* Read data from SPI FLASH memory */
        ReadBuffer(Rx_Buffer, FLASH_TestAddress, BufferSize);

        /* Check the correctness of written data */
        for (Index = 0; Index < BufferSize; Index++)
        {
            if (Tx_Buffer[Index] != Rx_Buffer[Index])
            {
                //FAILED : Transmitted and Received data by SPI are different
                TestStatus = -1;
            }
            else
            {
                //PASSED : Transmitted and Received data by SPI are same
                TestStatus = 0;
            }
        }

        /* Perform an erase in the Flash followed by a read of the written data */
        /* Erase SPI FLASH Sector to write on */
        EraseSector(FLASH_TestAddress);

        /* Read data from SPI FLASH memory */
        ReadBuffer(Rx_Buffer, FLASH_TestAddress, BufferSize);
        /* Check the correctness of erasing operation data */
        for (Index = 0; Index < BufferSize; Index++)
        {
            if (Rx_Buffer[Index] != 0xFF)
            {
                //FAILED : Specified sector part is not well erased
                TestStatus = -1;
            }
            else
            {
                //PASSED : Specified sector part is erased
                TestStatus = 0;
            }
        }
    }

    return TestStatus;
}
