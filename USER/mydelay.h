#ifndef _MYDELAY_H
#define _MYDELAY_H

#include "stm32f4xx.h"

void Delay_us(unsigned long nTime);

void Delay_ms(unsigned long nTime);

void board_init(void);

int get_tick_count(unsigned long *count);

#endif
