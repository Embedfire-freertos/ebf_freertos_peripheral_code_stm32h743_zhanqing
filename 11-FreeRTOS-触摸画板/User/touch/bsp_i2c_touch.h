#ifndef __I2C_TOUCH_H
#define	__I2C_TOUCH_H

#include "stm32h7xx.h"

/*设定使用的电容屏IIC设备地址*/
#define GTP_ADDRESS            0xBA

#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

#define GTP_I2C_SCL_CLK()          __GPIOH_CLK_ENABLE()
#define GTP_I2C_SDA_CLK()          __GPIOH_CLK_ENABLE()
#define GTP_I2C_RST_CLK()          __GPIOD_CLK_ENABLE()
#define GTP_I2C_GTP_CLK()          __GPIOG_CLK_ENABLE()

/*I2C引脚*/
#define GTP_I2C_SCL_PIN                  GPIO_PIN_4                 
#define GTP_I2C_SCL_GPIO_PORT            GPIOH                       

#define GTP_I2C_SDA_PIN                  GPIO_PIN_5                 
#define GTP_I2C_SDA_GPIO_PORT            GPIOH                     

/*复位引脚*/
#define GTP_RST_GPIO_PORT                GPIOD
#define GTP_RST_GPIO_PIN                 GPIO_PIN_6
/*中断引脚*/
#define GTP_INT_GPIO_PORT                GPIOG
#define GTP_INT_GPIO_PIN                 GPIO_PIN_8
#define GTP_INT_EXTI_IRQ                 EXTI9_5_IRQn
/*中断服务函数*/
#define GTP_IRQHandler                   EXTI9_5_IRQHandler

#define	digitalHigh(p,i)			{p->BSRRL=i;}			  //设置为高电平		
#define digitalLow(p,i)			  {p->BSRRH=i;}				//输出低电平

//软件IIC使用的宏
/* SCL = 1 */
#define I2C_SCL_1()  digitalHigh(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN)
/* SCL = 0 */
#define I2C_SCL_0()  digitalLow(GTP_I2C_SCL_GPIO_PORT, GTP_I2C_SCL_PIN)		
/* SDA = 1 */
#define I2C_SDA_1()  digitalHigh(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)		
/* SDA = 0 */
#define I2C_SDA_0()  digitalLow(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)		
/* 读SDA口线状态 */
#define I2C_SDA_READ()   HAL_GPIO_ReadPin(GTP_I2C_SDA_GPIO_PORT, GTP_I2C_SDA_PIN)	

//函数接口
void I2C_Touch_Init(void);
uint32_t I2C_WriteBytes(uint8_t ClientAddr,uint8_t* pBuffer,  uint8_t NumByteToWrite);
uint32_t I2C_ReadBytes(uint8_t ClientAddr,uint8_t* pBuffer, uint16_t NumByteToRead);
void I2C_ResetChip(void);
void I2C_GTP_IRQDisable(void);
void I2C_GTP_IRQEnable(void);

#endif /* __I2C_TOUCH_H */
