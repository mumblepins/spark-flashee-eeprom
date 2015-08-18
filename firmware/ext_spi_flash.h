/**
  ******************************************************************************
  * @file    ext_spi_flash.h
  * @author  Daniel Sullivan
  * @version V1.0.0
  * @date    11-Aug-2015
  * @brief   Header for external spi flash driver  (modeled after spi_flash.h
             from particle.io)
  ******************************************************************************
  Copyright (c) 2013-2015 Particle Industries, Inc.  All rights reserved.


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, see <http://www.gnu.org/licenses/>.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXT_SPI_FLASH_H
#define __EXT_SPI_FLASH_H
#include "application.h"

/* SST25 SPI Flash supported commands */
#define extFLASH_CMD_RDSR                 0x05        /* Read Status Register */
#define extFLASH_CMD_WRSR                 0x01        /* Write Status Register */
#define extFLASH_CMD_EWSR                 0x50        /* Write Enable Status */
#define extFLASH_CMD_WRDI                 0x04        /* Write Disable */
#define extFLASH_CMD_WREN                 0x06        /* Write Enable */
#define extFLASH_CMD_READ                 0x03        /* Read Data Bytes */
#define extFLASH_CMD_WRITE                0x02        /* Byte Program */
#define extFLASH_CMD_SE                   0x20        /* 4KB Sector Erase instruction */
#define extFLASH_CMD_BE                   0xC7        /* Bulk Chip Erase instruction */
#define extFLASH_CMD_RDID                 0x9F        /* JEDEC ID Read */
#define extFLASH_WIP_FLAG                 0x01        /* Write In Progress (WIP) flag */
#define extFLASH_DUMMY_BYTE               0xA5

#define extFLASH_PROGRAM_PAGESIZE         0x100       /* 256 bytes */



#ifdef SPARK  // Particle Devices
    #if PLATFORM_ID < 3  // Particle Core
        #define DEF_CLOCK_DIVIDER SPI_CLOCK_DIV2
    #else // Particle Photon or P1
        #define DEF_CLOCK_DIVIDER SPI_CLOCK_DIV4
    #endif
#else // default to a slow specified
    #define DEF_CLOCK_DIVIDER SPI_CLOCK_DIV16
#endif

class ExtSpiFlash
{
public:
    ExtSpiFlash(void);
    ExtSpiFlash(uint32_t jedecID, uint16_t csPin, uint32_t pageSize, uint32_t pageCount, uint8_t clockDivider = DEF_CLOCK_DIVIDER);
    void Init(void);
    void EraseSector(uint32_t SectorAddr);
    void EraseBulk(void);
    void WriteBuffer(const uint8_t *pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);
    void ReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
    uint32_t ReadID(void);
    /* Flash Self Test Routine */
    int SelfTest(void);

    uint32_t pageSize(void);
    uint32_t pageCount(void);

private:
    uint32_t _jedec_id = 0xC22014; /* JEDEC Read-ID Data  (Defaults to MX25L8006E) */
    uint16_t _cs_pin = A2; /* Defaults to Standard SPI Chip Select (A2) */
    uint32_t _page_size = 0x1000;
    uint32_t _page_count = 512;
    uint8_t _clock_divider = DEF_CLOCK_DIVIDER;
    void SPI_Init(void);
    void CS_LOW(void);
    void CS_HIGH(void);
    void WritePage(const uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);
    void WriteEnable(void);
    void WriteDisable(void);
    void WaitForWriteEnd(void);
    uint8_t SendByte(uint8_t byte);

};







#endif /* __SPI_FLASH_H */
