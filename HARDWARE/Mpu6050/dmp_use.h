#ifndef _DMP_USE_H
#define _DMP_USE_H

#include "stm32f4xx.h"

void gyro_data_ready_cb(void);

u8 Mpu_Dmp_Init(void);

u8 Get_Dmp_Data(void);

#endif
