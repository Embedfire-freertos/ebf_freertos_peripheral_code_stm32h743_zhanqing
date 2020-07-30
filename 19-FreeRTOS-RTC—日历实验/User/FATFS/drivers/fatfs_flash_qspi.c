 /**
  ******************************************************************************
  * @file    bsp_xxx.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   QSPI flash �ײ�Ӧ�ú���bsp 
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��STM32H743 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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
	if(1)			/*���FLASH�Ƿ���������*/
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
			*(DWORD * )buff = 4096;	/* ����������2560*4096/1024/1024=10(MB) */
		break;
		case GET_SECTOR_SIZE :     /*��ȡ������д�Ĵ�С���֣�*/

			*(WORD * )buff = 4096;	/*flash��Сд��ԪΪҳ��256�ֽڣ��˴�ȡ2ҳΪһ����д��λ*/
		break;
		case GET_BLOCK_SIZE :      /* ͬʱ��������������˫�֣� */
			*(DWORD * )buff = 1;   /*flash��1��sectorΪ��С������λ*/
		break;
		case CTRL_TRIM:	
		break;
		case CTRL_SYNC :
		break;
	}
	return RES_OK;
}

DRESULT TM_FATFS_FLASH_SPI_disk_read(
										BYTE lun,//��������������豸ʱ�õ�(0...)
										BYTE *buff,//���ݻ����� 
										DWORD sector, //�����׵�ַ
										UINT count)//��������(1..128)
{
	if ((TM_FATFS_FLASH_SPI_Stat & STA_NOINIT)) 
	{
		return RES_NOTRDY;
	}
	sector+=4096;//����ƫ�ƣ��ⲿFlash�ļ�ϵͳ�ռ�����ⲿFlash����6M�ռ�
	SPI_FLASH_BufferRead(buff, sector <<12, count<<12);
	return RES_OK;
}

DRESULT TM_FATFS_FLASH_SPI_disk_write(
										BYTE lun,//��������������豸ʱ�õ�(0...)
										const BYTE *buff,//���ݻ����� 
										DWORD sector, //�����׵�ַ
										UINT count)//��������(1..128)
{
	uint32_t write_addr;  
	sector+=4096;//����ƫ�ƣ��ⲿFlash�ļ�ϵͳ�ռ�����ⲿFlash����4M�ռ�
	write_addr = sector<<12;    
	SPI_FLASH_SectorErase(write_addr);
	SPI_FLASH_BufferWrite((uint8_t*)buff,write_addr,4096);
	return RES_OK;
}
   
/*********************************************END OF FILE**********************/
