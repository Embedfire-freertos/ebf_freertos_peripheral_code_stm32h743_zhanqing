#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "stm32h7xx.h"

#define ADVANCE_TIM           		TIM1
#define ADVANCE_TIM_CLK_ENABLE()  __HAL_RCC_TIM1_CLK_ENABLE()

#define ADVANCE_TIM_IRQn		    	TIM1_UP_TIM10_IRQn
#define ADVANCE_TIM_IRQHandler    TIM1_UP_TIM10_IRQHandler

#define TIM_DMA_BUFSIZE               3
/* 高级定时器PWM输出 */
/* PWM输出引脚 */
#define ADVANCE_OCPWM_PIN              		GPIO_PIN_9            
#define ADVANCE_OCPWM_GPIO_PORT        		GPIOE                      
#define ADVANCE_OCPWM_GPIO_CLK_ENABLE()  	__GPIOE_CLK_ENABLE()
#define ADVANCE_OCPWM_AF									GPIO_AF1_TIM1

/* DMA  */
#define TIM_DMA												DMA2
#define TIM_DMA_ADDRESS               &TIM1
#define TIM_DMA_BUFSIZE               3
#define TIM_DMA_CLK                   __HAL_RCC_DMA2_CLK_ENABLE()
#define TIM_DMA_CHANNEL               DMA_REQUEST_TIM1_CH1
#define TIM_DMA_STREAM                DMA2_Stream3

extern TIM_HandleTypeDef TIM_TimeBaseStructure;
extern DMA_HandleTypeDef TIM_DMA_HANDLE;

void TIMx_Configuration(void);

#endif /* __ADVANCE_TIM_H */

