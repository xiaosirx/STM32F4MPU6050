#include "mpu6050.h"


/**
*@breif:��Ĵ���д�루�ֽڣ�
*@param:reg		�Ĵ���
		data 	����
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
*@breif:�ӼĴ����������ֽڣ�
*@param:reg 	�Ĵ���
*@retval:val 	��������ֵ
**/
u8 Mpu_RD_Reg(u8 reg)
{
	u8 val;
	I2C_Start();
	I2C_WR_Byte(I2C_ADDR);
	I2C_WR_Byte(reg);
	
	I2C_Start();
	I2C_WR_Byte(I2C_ADDR + 1);	//�ı䷽��
	val = I2C_RD_Byte();
	I2C_Ack();
	I2C_Stop();
	return val;
}

u8 Mpu_Init()
{
	IIC_Init();
	
	Mpu_WR_Reg(PWR_MGMT_1, 0x80);		//�������мĴ���
	Delay_us(50);
	Mpu_WR_Reg(PWR_MGMT_1, 0x00);		//������������״̬		
	Mpu_WR_Reg(GYRO_CONFIG, 0x08);		//���������������	��500 ��/s
	Mpu_WR_Reg(ACCEL_CONFIG, 0x08);		//���ٶȼ����������	��4g
	Mpu_WR_Reg(INT_ENABLE, 0x00);		//�ر������ж�
	Mpu_WR_Reg(USER_CTRL, 0x00);		//�ر�MPU��ΪI2C����ģʽ
	Mpu_WR_Reg(FIFO_EN, 0x00);			//�ر�FIFO
	Mpu_WR_Reg(SMPLRT_DIV, 20 - 1);		//������Ƶ20 	Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	Mpu_WR_Reg(CONFIG, 0x04);			//��ͨ�˲���		����Sample Rate = 1k / 20 = 50,�˲�Ƶ�ʴ���Ϊ����Ƶ�ʵ�һ��
	Mpu_WR_Reg(PWR_MGMT_1, 0x01);		//����ʱ��		ѡ��������x��PLL��Ϊʱ��Դ
	Mpu_WR_Reg(PWR_MGMT_2, 0x00);		//�򿪼��ٶȴ�������������
	
	return Mpu_RD_Reg(WHO_AM_I);
}
