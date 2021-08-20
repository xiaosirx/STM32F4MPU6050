#include "mpu6050.h"


/**
*@breif:向寄存器写入（字节）
*@param:reg		寄存器
		data 	数据
*@retval:None
**/
void Mpu_WR_Reg(u8 reg, u8 data)
{
	I2C_Start();
	I2C_WR_Byte(I2C_ADDR);
	I2C_WR_Byte(reg);
	I2C_WR_Byte(data);
	I2C_Stop();
}

/**
*@breif:从寄存器读出（字节）
*@param:reg 	寄存器
*@retval:val 	读出来的值
**/
u8 Mpu_RD_Reg(u8 reg)
{
	u8 val;
	I2C_Start();
	I2C_WR_Byte(I2C_ADDR);
	I2C_WR_Byte(reg);
	
	I2C_Start();
	I2C_WR_Byte(I2C_ADDR + 1);	//改变方向
	val = I2C_RD_Byte();
	I2C_Ack();
	I2C_Stop();
	return val;
}

u8 Mpu_Init()
{
	IIC_Init();
	
	Mpu_WR_Reg(PWR_MGMT_1, 0x80);		//重置所有寄存器
	Delay_us(50);
	Mpu_WR_Reg(PWR_MGMT_1, 0x00);		//进入正常工作状态		
	Mpu_WR_Reg(GYRO_CONFIG, 0x08);		//陀螺仪最大满量程	±500 °/s
	Mpu_WR_Reg(ACCEL_CONFIG, 0x08);		//加速度计最大满量程	±4g
	Mpu_WR_Reg(INT_ENABLE, 0x00);		//关闭所有中断
	Mpu_WR_Reg(USER_CTRL, 0x00);		//关闭MPU作为I2C主机模式
	Mpu_WR_Reg(FIFO_EN, 0x00);			//关闭FIFO
	Mpu_WR_Reg(SMPLRT_DIV, 20 - 1);		//采样分频20 	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	Mpu_WR_Reg(CONFIG, 0x04);			//低通滤波器		这里Sample Rate = 1k / 20 = 50,滤波频率大致为采样频率的一半
	Mpu_WR_Reg(PWR_MGMT_1, 0x01);		//设置时钟		选择陀螺仪x轴PLL作为时钟源
	Mpu_WR_Reg(PWR_MGMT_2, 0x00);		//打开加速度传感器和陀螺仪
	
	return Mpu_RD_Reg(WHO_AM_I);
}
