/**
  ******************************************************************************
  * @file    bsp_advance_tim.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   高级控制定时器定时范例
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火  STM32 FH743 开发板  
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "./tim/bsp_advance_tim.h"
TIM_HandleTypeDef TIM_TimeBaseStructure;

DMA_HandleTypeDef TIM_DMA_HANDLE;

DMA_InitTypeDef	TIM_DMA_InitTypeDef;

uint16_t aSRC_Buffer[TIM_DMA_BUFSIZE] ={383, 767, 127};

static void TIM_DMA_Config(void);
/**
  * @brief  配置TIM复用输出PWM时用到的I/O
  * @param  无
  * @retval 无
  */
static void TIMx_GPIO_Config(void) 
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启定时器相关的GPIO外设时钟*/
	ADVANCE_OCPWM_GPIO_CLK_ENABLE(); 

	/* 定时器功能引脚初始化 */
	/* 高级定时器输入捕获引脚 */
	GPIO_InitStructure.Pin = ADVANCE_OCPWM_PIN;	
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;    
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; 	
	GPIO_InitStructure.Alternate = ADVANCE_OCPWM_AF;
	HAL_GPIO_Init(ADVANCE_OCPWM_GPIO_PORT, &GPIO_InitStructure);

}

 /**
  * @brief  高级控制定时器 TIMx,x[1,8]中断优先级配置
  * @param  无
  * @retval 无
  */
static void TIMx_NVIC_Configuration(void)
{	
	//设置抢占优先级，子优先级
	HAL_NVIC_SetPriority(ADVANCE_TIM_IRQn, 6, 0);
	// 设置中断来源
	HAL_NVIC_EnableIRQ(ADVANCE_TIM_IRQn);
	//设置抢占优先级，子优先级
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
	// 设置中断来源
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/*
 * 注意：TIM_TimeBaseInitTypeDef结构体里面有5个成员，TIM6和TIM7的寄存器里面只有
 * TIM_Prescaler和TIM_Period，所以使用TIM6和TIM7的时候只需初始化这两个成员即可，
 * 另外三个成员是通用定时器和高级定时器才有.
 *-----------------------------------------------------------------------------
 * TIM_Prescaler         都有
 * TIM_CounterMode			 TIMx,x[6,7]没有，其他都有（基本定时器）
 * TIM_Period            都有
 * TIM_ClockDivision     TIMx,x[6,7]没有，其他都有(基本定时器)
 * TIM_RepetitionCounter TIMx,x[1,8]才有(高级定时器)
 *-----------------------------------------------------------------------------
 */
static void TIM_Mode_Config(void)
{

	// 开启TIMx_CLK,x[1,8] 
	ADVANCE_TIM_CLK_ENABLE(); 
	
	TIM_TimeBaseStructure.Instance = ADVANCE_TIM;
	/* 累计 TIM_Period个后产生一个更新或者中断*/		
	//当定时器从0计数到4999，即为5000次，为一个定时周期
	TIM_TimeBaseStructure.Init.Period =5000-1;       
	// 高级控制定时器时钟源TIMxCLK = HCLK=216MHz 
	// 设定定时器频率为=TIMxCLK/(TIM_Prescaler+1)=10000Hz
	TIM_TimeBaseStructure.Init.Prescaler = 24000-1;	
	// 采样时钟分频
	TIM_TimeBaseStructure.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	// 计数方式
	TIM_TimeBaseStructure.Init.CounterMode=TIM_COUNTERMODE_UP;
	// 重复计数器:重复1次，即计数两次才生成一个中断
	TIM_TimeBaseStructure.Init.RepetitionCounter=3;
	
	TIM_TimeBaseStructure.hdma[TIM_DMA_ID_CC1] = &TIM_DMA_HANDLE;
	
	// 初始化定时器TIMx, x[1,8]
	HAL_TIM_PWM_Init(&TIM_TimeBaseStructure);

	TIM_DMA_Config();

	TIM_OC_InitTypeDef TIM_OC_Init;
	/*PWM模式配置*/
	//配置为PWM模式1
	TIM_OC_Init.OCMode = TIM_OCMODE_PWM1;
	TIM_OC_Init.Pulse = 383;
	TIM_OC_Init.OCPolarity = TIM_OCPOLARITY_HIGH;
	TIM_OC_Init.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	TIM_OC_Init.OCIdleState = TIM_OCIDLESTATE_SET;
	TIM_OC_Init.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
	//初始化通道1输出PWM 
	HAL_TIM_PWM_ConfigChannel(&TIM_TimeBaseStructure,&TIM_OC_Init,TIM_CHANNEL_1);

}

static void TIM_DMA_Config(void)
{
	TIM_DMA_CLK;
	
	TIM_DMA_HANDLE.Instance 									= TIM_DMA_STREAM;
	TIM_DMA_HANDLE.Init.Request								= TIM_DMA_CHANNEL;
	TIM_DMA_HANDLE.Init.Direction							= DMA_MEMORY_TO_PERIPH;
	TIM_DMA_HANDLE.Init.FIFOMode							= DMA_FIFOMODE_DISABLE;
	TIM_DMA_HANDLE.Init.MemDataAlignment			= DMA_MDATAALIGN_HALFWORD;
	TIM_DMA_HANDLE.Init.MemInc								= DMA_MINC_ENABLE;
	TIM_DMA_HANDLE.Init.Mode									= DMA_CIRCULAR;
	TIM_DMA_HANDLE.Init.PeriphDataAlignment		= DMA_PDATAALIGN_HALFWORD;
	TIM_DMA_HANDLE.Init.PeriphInc							= DMA_PINC_DISABLE;
	TIM_DMA_HANDLE.Init.Priority							= DMA_PRIORITY_HIGH;
	
	HAL_DMA_Init(TIM_TimeBaseStructure.hdma[TIM_DMA_ID_CC1]);
	
	__HAL_LINKDMA(&TIM_TimeBaseStructure,hdma[TIM_DMA_ID_CC1],TIM_DMA_HANDLE);

}
/**
  * @brief  初始化高级控制定时器定时，1s产生一次中断
  * @param  无
  * @retval 无
  */
void TIMx_Configuration(void)
{
	TIMx_GPIO_Config();
	TIMx_NVIC_Configuration();
	TIM_Mode_Config();	
	HAL_TIM_PWM_Start_DMA(&TIM_TimeBaseStructure,TIM_CHANNEL_1,(uint32_t *)aSRC_Buffer,3);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//接收DMA传输数据，并完成PWM输出回调
}
/*********************************************END OF FILE**********************/
