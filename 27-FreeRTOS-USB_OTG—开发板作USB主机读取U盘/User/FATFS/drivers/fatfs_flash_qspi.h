#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#include "stm32h7xx.h"
#include <stdio.h>

#include "diskio.h"
#include "integer.h"
#include "ff_gen_drv.h"

/* Private typedef -----------------------------------------------------------*/
//#define  sFLASH_ID                       0xEF3015     //W25X16
//#define  sFLASH_ID                       0xEF4015	    //W25Q16
//#define  sFLASH_ID                       0XEF4017     //W25Q64
#define  sFLASH_ID                         0XEF4018     //W25Q128

/* QSPI Error codes */
#define QSPI_OK            ((uint8_t)0x00)
#define QSPI_ERROR         ((uint8_t)0x01)
#define QSPI_BUSY          ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint8_t)0x04)
#define QSPI_SUSPENDED     ((uint8_t)0x08)

DSTATUS TM_FATFS_FLASH_SPI_disk_initialize(BYTE lun);
DSTATUS TM_FATFS_FLASH_SPI_disk_status(BYTE lun) ;
DRESULT TM_FATFS_FLASH_SPI_disk_ioctl(BYTE lun,BYTE cmd, void *buff) ;
DRESULT TM_FATFS_FLASH_SPI_disk_read(BYTE lun,BYTE *buff, DWORD sector, UINT count) ;
DRESULT TM_FATFS_FLASH_SPI_disk_write(BYTE lun,const BYTE *buff, DWORD sector, UINT count) ;

#endif /* __SPI_FLASH_H */

