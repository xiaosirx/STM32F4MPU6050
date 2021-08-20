#include "iic.h"

void IIC_Init()
{
	GPIO_InitTypeDef GpioIniture;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOA, ENABLE);
	
	GpioIniture.GPIO_Mode  = GPIO_Mode_OUT;
	GpioIniture.GPIO_OType = GPIO_OType_PP;
	GpioIniture.GPIO_Pin   = GPIO_Pin_8;
	GpioIniture.GPIO_PuPd  = GPIO_PuPd_UP;
	GpioIniture.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA, &GpioIniture);

	GpioIniture.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GpioIniture);

	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	GPIO_SetBits(GPIOC, GPIO_Pin_9);
}

void I2C_Start()
{
	SCLH;
	SDAH;
	Delay_us(delay_time);
	SDAL;
	Delay_us(delay_time);
	SCLL;
}


void I2C_Stop()
{
	SDAL;
	Delay_us(delay_time);
	SCLH;	
	Delay_us(delay_time);
	SDAH;
	Delay_us(delay_time);
}


void I2C_No_Ack()
{
	Delay_us(delay_time);
	SDAH;
	SCLH;
	Delay_us(delay_time);
	SCLL;
	Delay_us(delay_time);
	SDAL;
	Delay_us(delay_time);
}

void I2C_Ack()
{
	Delay_us(delay_time);
	SDAL;
	SCLH;
	Delay_us(delay_time);
	SCLL;
	Delay_us(delay_time);
	SDAL;
	Delay_us(delay_time);
}

/**
*@breif:写字节
*@param:data 写入的数据
*@retval:res 	1(nack)	0(ack)
**/
u8 I2C_WR_Byte(u8 data)
{
	u8 i, res;
	for(i = 0; i < 8; i ++)
	{
		SCLL;
		if((data >> (7 - i)) & 0x01)
			SDAH;
		else 
			SDAL;
		Delay_us(delay_time);
		SCLH;
		Delay_us(delay_time);
	}
	SDA_IN();
	SCLL;
	Delay_us(delay_time);
	SCLH;
	Delay_us(delay_time);
	if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9))
		res = 1;
	else 
		res = 0;
	SCLL;
	SDA_OUT();
	return res;
}

/**
*@breif:读字节
*@param:None
*@retval:temp 读到的数据
**/
u8 I2C_RD_Byte()
{
	u8 temp = 0, i;
	SDA_IN();
	for(i = 0;i < 8;i ++)
	{
		Delay_us(delay_time);
		SCLH;
		Delay_us(delay_time);
		temp <<= 1;
		if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9))
			temp ++;
		Delay_us(delay_time);
		SCLL;
	}
	SDA_OUT();
	return temp;
}


/**
*@brief:发送多个数据到一个寄存器
*@prarm:slave_addr	从机地址
*		reg_addr	寄存器地址
*		length		数据个数
*		data		数据地址
*@retval:res	0(写入成功)	1（写入失败）
**/
u8 Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char const *data)
	   {
			u8 i, res;
			I2C_Start();
			res = I2C_WR_Byte(slave_addr<<1);
			res = I2C_WR_Byte(reg_addr);
			for(i = 0; i < length; i ++)
			{
				res = I2C_WR_Byte(*data);
				data ++;
			}
			I2C_Stop();
			return res;
	   }
	
/**
*@brief:从一个寄存器读多个数据
*@prarm:slave_addr	从机地址
*		reg_addr	寄存器地址
*		length		数据个数
*		data		数据存放地址
*@retval:res	0(写入成功)	1（写入失败）
**/	   
u8 Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,
       unsigned char length, unsigned char *data)
	   {
			u8 i, res;
			I2C_Start();
			res = I2C_WR_Byte(slave_addr<<1);
			res = I2C_WR_Byte(reg_addr);
		   
			I2C_Start();
			res = I2C_WR_Byte((slave_addr<<1) + 1);
			for(i = 0; i < length; i ++)
			{
				if(i == length - 1){
					*data = I2C_RD_Byte();
					I2C_No_Ack();
				}
				else{
					*data = I2C_RD_Byte();
					I2C_Ack();
					data ++;
				}
				
			}
			I2C_Stop();
			return res;
	   }
	   
