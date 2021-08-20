
/********极其精确的延时*********/

#include "mydelay.h"

/**
*@brief:systick中断延时与时间戳
**/	   
#define TICK_FREQ (1000u)

volatile uint32_t g_ul_ms_ticks=0;
static volatile uint32_t TimingDelay=0;
unsigned long idle_time=0;
extern uint32_t SystemCoreClock; //168000000=168Mhz (original value)

void board_init() 
{	
	SystemCoreClockUpdate();                               // Update the system clock variable (might not have been set before)
                                                           // With this call, the core clock gets set to 56MHz
  
	/* Enable PWR APB1 Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	/* Allow access to Backup */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset RTC Domain */
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
}

void Delay_us(unsigned long nTime)
{
	if (SysTick_Config (SystemCoreClock / TICK_FREQ / TICK_FREQ)) {     // Setup SysTick Timer for 1 usec interrupts
		while (1);                                          // Handle Error
	}
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

void Delay_ms(unsigned long nTime)
{
	if (SysTick_Config (SystemCoreClock / TICK_FREQ)) {     // Setup SysTick Timer for 1 msec interrupts
		while (1);                                          // Handle Error
	}	
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

int get_tick_count(unsigned long *count)
{
    count[0] = g_ul_ms_ticks;
	return 0;
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
		TimingDelay--;
}

void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}

void SysTick_Handler(void)
{
  	TimingDelay_Decrement();
    TimeStamp_Increment();
}
