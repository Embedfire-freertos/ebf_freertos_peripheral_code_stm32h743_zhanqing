#ifndef __BEEP_H
#define	__BEEP_H


#include "stm32h7xx.h"
#include "./led/bsp_led.h"   

/* 定义蜂鸣器连接的GPIO端口, 用户只需要修改下面的代码即可改变控制的蜂鸣器引脚 */
#define BEEP_GPIO_PORT    	    GPIOG		              /* GPIO端口 */
#define BEEP_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOG_CLK_ENABLE()  /* GPIO端口时钟 */
#define BEEP_GPIO_PIN		  	GPIO_PIN_7			          /* 连接到蜂鸣器的GPIO */



/* 带参宏，可以像内联函数一样使用 */
#define BEEP(a)	HAL_GPIO_WritePin(BEEP_GPIO_PORT,BEEP_GPIO_PIN,a)
				


/* 定义控制IO的宏 */
#define BEEP_TOGGLE		digitalToggle(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_ON	        digitalHi(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_OFF		digitalLo(BEEP_GPIO_PORT,BEEP_GPIO_PIN)

void BEEP_GPIO_Config(void);
					
#endif /* __BEEP_H */
