#ifndef _IIC_H
#define _IIC_H

#include "stm32f4xx.h"
#include "mydelay.h"

#define SCLH	GPIO_SetBits(GPIOA, GPIO_Pin_8)
#define SCLL	GPIO_ResetBits(GPIOA, GPIO_Pin_8)		
#define SDAH	GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define SDAL	GPIO_ResetBits(GPIOC, GPIO_Pin_9)		

#define SDA_OUT()	{GPIOC->MODER&=~(3<<(9*2));GPIOC->MODER|=1<<9*2;}										
#define SDA_IN()    {GPIOC->MODER&=~(3<<(9*2));GPIOC->MODER|=0<<9*2;}										

#define	delay_time	5

void IIC_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_No_Ack(void);
void I2C_Ack(void);
u8 I2C_WR_Byte(u8 data);
u8 I2C_RD_Byte(void);

u8 Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char const *data);
u8 Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char *data);
				
#endif
