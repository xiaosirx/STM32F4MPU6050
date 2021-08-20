#ifndef _MPU6050_H
#define _MPU6050_H

#include "stm32f4xx.h"
#include "iic.h"

#define I2C_ADDR	0XD0		//mpu6050的八位I2C通信地址（AD0接地）

//寄存器定义		寄存器			地址
#define 		PWR_MGMT_1		0X6B	
#define			GYRO_CONFIG		0X1B
#define			ACCEL_CONFIG	0X1C
#define 		INT_ENABLE		0X38
#define			USER_CTRL		0X6A
#define 		FIFO_EN			0X23
#define			SMPLRT_DIV		0X19
#define			CONFIG			0X1A
#define 		PWR_MGMT_2		0X6C
#define 		ACCEL_XOUT_H	0X3B
#define 		ACCEL_XOUT_L	0X3C
#define			ACCEL_YOUT_H	0X3D
#define			ACCEL_YOUT_L	0X3E
#define			ACCEL_ZOUT_H	0X3F
#define			ACCEL_ZOUT_L	0X40
#define			TEMP_OUT_H		0X41
#define			TEMP_OUT_L		0X42
#define			GYRO_XOUT_H		0X43
#define			GYRO_XOUT_L		0X44
#define			GYRO_YOUT_H		0X45
#define 		GYRO_YOUT_L		0X46
#define			GYRO_ZOUT_H		0X47
#define			GYRO_ZOUT_L		0X48
#define			WHO_AM_I		0X75

void Mpu_WR_Reg(u8 reg, u8 data);
u8 Mpu_RD_Reg(u8 reg);
u8 Mpu_Init(void);

#endif
