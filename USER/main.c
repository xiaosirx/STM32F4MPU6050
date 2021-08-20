#include "stm32f4xx.h"
#include "usmart.h"
#include "usart.h"
#include "dmp_use.h"
#include "lcd.h"
#include "mydelay.h"
#include "iic.h"

extern float pitch, yaw, roll;

int main()
{
	IIC_Init();	
	board_init();
	Delay_us(10);
	Mpu_Dmp_Init();
	LCD_Init();
	LCD_DisplayOn();
	LCD_Clear(WHITE);
	
	LCD_ShowString(4, 1, 8 * 11, 1, 16, "pitch:    .");
	LCD_ShowString(4, 17, 8 * 10, 1, 16, "roll:    .");
	LCD_ShowString(4, 33, 8 * 9, 1, 16, "yaw:    .");
	
	while(1)
	{
		gyro_data_ready_cb();		//在读取欧拉角之前务必调用此函数
		while(Get_Dmp_Data());
		LCD_ShowxNum(60, 1, (int)pitch, 3, 16, 0);
		LCD_ShowxNum(92, 1, (int)((pitch - (int)(pitch)) * 1000), 3, 16, 0);
		LCD_ShowxNum(52, 17, (int)roll, 3, 16, 0);
		LCD_ShowxNum(84, 17, (int)((roll - (int)(roll)) * 1000), 3, 16, 0);
		LCD_ShowxNum(44, 33, (int)yaw, 3, 16, 0);
		LCD_ShowxNum(76, 33, (int)((yaw - (int)(yaw)) * 1000), 3, 16, 0);
	}
}
