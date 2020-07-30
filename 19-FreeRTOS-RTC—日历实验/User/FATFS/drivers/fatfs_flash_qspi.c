 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   QSPI flash 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火STM32H743 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "fatfs_flash_qspi.h"
#include "ff_gen_drv.h"
#include "bsp_spi_flash.h"
extern void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);
extern void SPI_FLASH_SectorErase(uint32_t SectorAddr);
extern void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
static volatile DSTATUS TM_FATFS_FLASH_SPI_Stat = STA_NOINIT;	/* Physical drive status */

const Diskio_drvTypeDef  SPI_Driver =
{
  TM_FATFS_FLASH_SPI_disk_initialize,
  TM_FATFS_FLASH_SPI_disk_status,
  TM_FATFS_FLASH_SPI_disk_read, 
#if  _USE_WRITE == 1
  TM_FATFS_FLASH_SPI_disk_write,
#endif /* _USE_WRITE == 1 */
  
#if  _USE_IOCTL == 1
  TM_FATFS_FLASH_SPI_disk_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
DSTATUS TM_FATFS_FLASH_SPI_disk_initialize(BYTE lun)
{

	return TM_FATFS_FLASH_SPI_disk_status(NULL);

}
/**
  * @brief  Initializes the QSPI interface.
  * @retval QSPI memory status
  */
uint8_t BSP_QSPI_Init(void)
{ 

  return QSPI_OK;
}


DSTATUS TM_FATFS_FLASH_SPI_disk_status(BYTE lun)
{
	if(1)			/*检测FLASH是否正常工作*/
	{
		return TM_FATFS_FLASH_SPI_Stat &= ~STA_NOINIT;	/* Clear STA_NOINIT flag */
	}
	else
	{
		return TM_FATFS_FLASH_SPI_Stat |= STA_NOINIT;
	}
}

DRESULT TM_FATFS_FLASH_SPI_disk_ioctl(BYTE lun,BYTE cmd, void *buff)
{
	switch (cmd) {
		case GET_SECTOR_COUNT:
			*(DWORD * )buff = 4096;	/* 扇区数量：2560*4096/1024/1024=10(MB) */
		break;
		case GET_SECTOR_SIZE :     /*获取扇区读写的大小（字）*/

			*(WORD * )buff = 4096;	/*flash最小写单元为页，256字节，此处取2页为一个读写单位*/
		break;
		case GET_BLOCK_SIZE :      /* 同时擦除扇区个数（双字） */
			*(DWORD * )buff = 1;   /*flash以1个sector为最小擦除单位*/
		break;
		case CTRL_TRIM:	
		break;
		case CTRL_SYNC :
		break;
	}
	return RES_OK;
}

DRESULT TM_FATFS_FLASH_SPI_disk_read(
										BYTE lun,//物理扇区，多个设备时用到(0...)
										BYTE *buff,//数据缓存区 
										DWORD sector, //扇区首地址
										UINT count)//扇区个数(1..128)
{
	if ((TM_FATFS_FLASH_SPI_Stat & STA_NOINIT)) 
	{
		return RES_NOTRDY;
	}
	sector+=4096;//扇区偏移，外部Flash文件系统空间放在外部Flash后面6M空间
	SPI_FLASH_BufferRead(buff, sector <<12, count<<12);
	return RES_OK;
}

DRESULT TM_FATFS_FLASH_SPI_disk_write(
										BYTE lun,//物理扇区，多个设备时用到(0...)
										const BYTE *buff,//数据缓存区 
										DWORD sector, //扇区首地址
										UINT count)//扇区个数(1..128)
{
	uint32_t write_addr;  
	sector+=4096;//扇区偏移，外部Flash文件系统空间放在外部Flash后面4M空间
	write_addr = sector<<12;    
	SPI_FLASH_SectorErase(write_addr);
	SPI_FLASH_BufferWrite((uint8_t*)buff,write_addr,4096);
	return RES_OK;
}
   
/*********************************************END OF FILE**********************/
