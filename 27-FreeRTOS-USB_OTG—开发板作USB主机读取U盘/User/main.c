/**
  *********************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   FreeRTOS V9.0.0  + STM32 �жϹ���
  *********************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  **********************************************************************
  */ 
 
/*
*************************************************************************
*                             ������ͷ�ļ�
*************************************************************************
*/ 
/* ������Ӳ��bspͷ�ļ� */
#include "stm32h7xx.h"
#include "main.h"
#include "bsp_led.h"
#include "bsp_key.h" 
#include "core_delay.h"   
#include "bsp_debug_usart.h"
#include "./flash/bsp_spi_flash.h"
#include "usb_host.h"
#include "usbh_diskio_dma.h"
/* FreeRTOSͷ�ļ� */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* FATFS ͷ�ļ� */
#include "ff.h"
#include "ff_gen_drv.h"
#include "string.h"
/* ��׼��ͷ�ļ� */
#include <string.h>

/**************************** ������ ********************************/
/* 
 * ��������һ��ָ�룬����ָ��һ�����񣬵����񴴽���֮�����;�����һ��������
 * �Ժ�����Ҫ��������������Ҫͨ�������������������������������Լ�����ô
 * ����������ΪNULL��
 */
 /* ���������� */
static TaskHandle_t AppTaskCreate_Handle;
/* LED������ */
static TaskHandle_t Test_Task_Handle = NULL;
/* KEY������ */
TaskHandle_t KEY_Task_Handle = NULL;
/********************************** �ں˶����� *********************************/
/*
 * �ź�������Ϣ���У��¼���־�飬�����ʱ����Щ�������ں˵Ķ���Ҫ��ʹ����Щ�ں�
 * ���󣬱����ȴ����������ɹ�֮��᷵��һ����Ӧ�ľ����ʵ���Ͼ���һ��ָ�룬������
 * �ǾͿ���ͨ��������������Щ�ں˶���
 *
 * �ں˶���˵���˾���һ��ȫ�ֵ����ݽṹ��ͨ����Щ���ݽṹ���ǿ���ʵ��������ͨ�ţ�
 * �������¼�ͬ���ȸ��ֹ��ܡ�������Щ���ܵ�ʵ��������ͨ��������Щ�ں˶���ĺ���
 * ����ɵ�
 * 
 */


/******************************* ȫ�ֱ������� ************************************/
/*
 * ��������дӦ�ó����ʱ�򣬿�����Ҫ�õ�һЩȫ�ֱ�����
 */
char USBDISKPath[4];
char scan_path[255] = "0:";
DIR dir; 
FIL fnew;													/* �ļ����� */
UINT fnum;            			  /* �ļ��ɹ���д���� */
BYTE ReadBuffer[1024]={0};        /* �������� */
BYTE WriteBuffer[] =              /* д������*/
"��ӭʹ��Ұ��STM32 H743ս�濪���� �����Ǹ������ӣ��½��ļ�ϵͳ�����ļ�\r\n";  
FATFS fs;

FRESULT res_sd;                /* �ļ�������� */


/*
*************************************************************************
*                             ��������
*************************************************************************
*/
static void AppTaskCreate(void);/* ���ڴ������� */
static void Test_Task(void* pvParameters);/* Test_Task����ʵ�� */
static void KEY_Task(void* pvParameters);/* KEY_Task����ʵ�� */
static void BSP_Init(void);/* ���ڳ�ʼ�����������Դ */
static FRESULT scan_files (char* path) ;
/*****************************************************************
  * @brief  ������
  * @param  ��
  * @retval ��
  * @note   ��һ����������Ӳ����ʼ�� 
            �ڶ���������APPӦ������
            ������������FreeRTOS����ʼ���������
  ****************************************************************/
int main(void)
{	
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  /* ������Ӳ����ʼ�� */
  BSP_Init();
	
  printf("����һ��[Ұ��]-STM32ȫϵ�п�����-FreeRTOS�̼������̣�\n\n");
  printf("����KEY1�������񣬰���KEY2�ָ����񣬲���SPI-FLASH\n");
	
   /* ���� AppTaskCreate ���� */
  xReturn = xTaskCreate((TaskFunction_t )AppTaskCreate,  /* ������ں��� */
                        (const char*    )"AppTaskCreate",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )1, /* ��������ȼ� */
                        (TaskHandle_t*  )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */ 
															
	if(pdFAIL != xReturn)/* �����ɹ� */
    vTaskStartScheduler();   /* �������񣬿������� */
  
  while(1);   /* ��������ִ�е����� */    
}


/***********************************************************************
  * @ ������  �� AppTaskCreate
  * @ ����˵���� Ϊ�˷���������е����񴴽����������������������
  * @ ����    �� ��  
  * @ ����ֵ  �� ��
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��ΪpdPASS */
  
  taskENTER_CRITICAL();           //�����ٽ���
  
  /* ����Test_Task���� */
  xReturn = xTaskCreate((TaskFunction_t )Test_Task, /* ������ں��� */
                        (const char*    )"Test_Task",/* �������� */
                        (uint16_t       )1024*2,   /* ����ջ��С */
                        (void*          )NULL,	/* ������ں������� */
                        (UBaseType_t    )2,	    /* ��������ȼ� */
                        (TaskHandle_t*  )&Test_Task_Handle);/* ������ƿ�ָ�� */
  if(pdPASS == xReturn)
    printf("����Test_Task����ɹ�!\r\n");
  /* ����KEY_Task���� */
  xReturn = xTaskCreate((TaskFunction_t )KEY_Task,  /* ������ں��� */
                        (const char*    )"KEY_Task",/* �������� */
                        (uint16_t       )512,  /* ����ջ��С */
                        (void*          )NULL,/* ������ں������� */
                        (UBaseType_t    )3, /* ��������ȼ� */
                        (TaskHandle_t*  )&KEY_Task_Handle);/* ������ƿ�ָ�� */ 
  if(pdPASS == xReturn)
    printf("����KEY_Task����ɹ�!\r\n");
  
  vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
  
  taskEXIT_CRITICAL();            //�˳��ٽ���
}


/**********************************************************************
  * @ ������  �� Test_Task
  * @ ����˵���� Test_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void Test_Task(void* parameter)
{	
	MX_USB_HOST_Init();
	LED1_ON;
	while (1)
	{
			vTaskDelay(50);   /* ��ʱ500��tick */
			MX_USB_HOST_Process();
		  LED2_TOGGLE;
	}
}

/**********************************************************************
  * @ ������  �� Test_Task
  * @ ����˵���� Test_Task��������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  ********************************************************************/
static void KEY_Task(void* parameter)
{	
	printf("��USB�����󣬰���Key1��ɨ��U�����ļ�\n");
	static char Already_Mount=0,Link_State=0;
	if( FATFS_LinkDriver(&USBH_Driver, USBDISKPath) == 0)
		Link_State = 1;

	//��ȡUSB����֪ͨ ,û��ȡ����һֱ�ȴ�
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

  while (1)
  {
		if( Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)  
		{
			if ( (1 == Link_State) && (0 == Already_Mount))
			{
					res_sd = f_mount(&fs,"0:",1);	/* Mount a logical drive */
				
					if(res_sd != FR_OK)
					{
						printf("\r\n�ļ�ϵͳ����ʧ�ܣ�\r\n");
						continue; 
					}
					Already_Mount = 1;
					scan_files(scan_path);
					res_sd = f_open(&fnew, "0:FatFs��д�����ļ�.txt",FA_CREATE_ALWAYS | FA_WRITE );
					if ( res_sd == FR_OK )
					{
						printf("����/����FatFs��д�����ļ�.txt�ļ��ɹ������ļ�д�����ݡ�\r\n");
						/* ��ָ���洢������д�뵽�ļ��� */
						res_sd=f_write(&fnew,WriteBuffer,sizeof(WriteBuffer),&fnum);
						if(res_sd==FR_OK)
						{
							printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n",fnum);
							printf("�����ļ�д�������Ϊ��\r\n%s\r\n",WriteBuffer);
						}
						else
						{
							printf("�����ļ�д��ʧ�ܣ�(%d)\n",res_sd);
						}    
						/* ���ٶ�д���ر��ļ� */
						f_close(&fnew);
					}
			}else if (Already_Mount == 1)
			{
				scan_files(scan_path);
			}else
			{
				printf("\r\n�ļ�ϵͳ����ʧ�ܹ���ʧ�ܣ�\r\n");
			}
		}
    vTaskDelay(20);/* ��ʱ20��tick */
  }
}
/***********************************************************************
  * @ ������  �� BSP_Init
  * @ ����˵���� �弶�����ʼ�������а����ϵĳ�ʼ�����ɷ��������������
  * @ ����    ��   
  * @ ����ֵ  �� ��
  *********************************************************************/
static void BSP_Init(void)
{
	/*
	 * STM32�ж����ȼ�����Ϊ4����4bit��������ʾ��ռ���ȼ�����ΧΪ��0~15
	 * ���ȼ�����ֻ��Ҫ����һ�μ��ɣ��Ժ������������������Ҫ�õ��жϣ�
	 * ��ͳһ��������ȼ����飬ǧ��Ҫ�ٷ��飬�мɡ�
	 */
	HAL_Init();	
	/* ��ʼ��ϵͳʱ�� */
	SystemClock_Config();
	
	/* �������ȼ�����Ϊ4 */
	HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
	
	/* LED ��ʼ�� */
	LED_GPIO_Config();
	
	/* KEY ��ʼ�� */
	Key_GPIO_Config();
	
	/* ���ڳ�ʼ��	*/
	DEBUG_USART_Config();

}


/**
  * @brief  scan_files �ݹ�ɨ��FatFs�ڵ��ļ�
  * @param  path:��ʼɨ��·��
  * @retval result:�ļ�ϵͳ�ķ���ֵ
  */
static FRESULT scan_files (char* path) 
{ 
  FRESULT res; 		//�����ڵݹ���̱��޸ĵı���������ȫ�ֱ���	
  FILINFO fno; 
  DIR dir; 
  int i;            
  char *fn;        // �ļ���	
	
#if _USE_LFN 
  /* ���ļ���֧�� */
  /* ����������Ҫ2���ֽڱ���һ�����֡�*/
  static char lfn[_MAX_LFN*2 + 1]; 	
  fno.lfname = lfn; 
  fno.lfsize = sizeof(lfn); 
#endif 
  //��Ŀ¼
  res = f_opendir(&dir, path); 
  if (res == FR_OK) 
	{ 
    i = strlen(path); 
    for (;;) 
		{ 
      //��ȡĿ¼�µ����ݣ��ٶ����Զ�����һ���ļ�
      res = f_readdir(&dir, &fno); 								
      //Ϊ��ʱ��ʾ������Ŀ��ȡ��ϣ�����
      if (res != FR_OK || fno.fname[0] == 0) break; 	
#if _USE_LFN 
      fn = *fno.lfname ? fno.lfname : fno.fname; 
#else 
      fn = fno.fname; 
#endif 
      //���ʾ��ǰĿ¼������			
      if (*fn == '.') continue; 	
      //Ŀ¼���ݹ��ȡ      
      if (fno.fattrib & AM_DIR)         
			{ 			
        //�ϳ�����Ŀ¼��        
        sprintf(&path[i], "/%s", fn); 		
        //�ݹ����         
        res = scan_files(path);	
        path[i] = 0;         
        //��ʧ�ܣ�����ѭ��        
        if (res != FR_OK) 
					break; 
      } 
			else 
			{ 
				printf("%s/%s\r\n", path, fn);								//����ļ���	
        /* ������������ȡ�ض���ʽ���ļ�·�� */        
      }//else
    } //for
  } 
  return res; 
}
 

/**
  * @brief  System Clock ����
  *         system Clock ��������: 
	*            System Clock source  = PLL (HSE)
	*            SYSCLK(Hz)           = 480000000 (CPU Clock)
	*            HCLK(Hz)             = 240000000 (AXI and AHBs Clock)
	*            AHB Prescaler        = 2
	*            D1 APB3 Prescaler    = 2 (APB3 Clock  120MHz)
	*            D2 APB1 Prescaler    = 2 (APB1 Clock  120MHz)
	*            D2 APB2 Prescaler    = 2 (APB2 Clock  120MHz)
	*            D3 APB4 Prescaler    = 2 (APB4 Clock  120MHz)
	*            HSE Frequency(Hz)    = 25000000
	*            PLL_M                = 5
	*            PLL_N                = 192
	*            PLL_P                = 2
	*            PLL_Q                = 4
	*            PLL_R                = 2
	*            VDD(V)               = 3.3
	*            Flash Latency(WS)    = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*ʹ�ܹ������ø��� */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

  /* ��������ʱ��Ƶ�ʵ������ϵͳƵ��ʱ����ѹ���ڿ����Ż����ģ�
		 ����ϵͳƵ�ʵĵ�ѹ����ֵ�ĸ��¿��Բο���Ʒ�����ֲᡣ  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
 
  /* ����HSE������ʹ��HSE��ΪԴ����PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
	/* ѡ��PLL��Ϊϵͳʱ��Դ����������ʱ�ӷ�Ƶ�� */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK  | \
																 RCC_CLOCKTYPE_HCLK    | \
																 RCC_CLOCKTYPE_D1PCLK1 | \
																 RCC_CLOCKTYPE_PCLK1   | \
                                 RCC_CLOCKTYPE_PCLK2   | \
																 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL3.PLL3M = 5;
  PeriphClkInitStruct.PLL3.PLL3N = 120;
  PeriphClkInitStruct.PLL3.PLL3P = 3;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1) { ; }
  }
	HAL_PWREx_EnableUSBVoltageDetector();
}
/****************************END OF FILE***************************/
