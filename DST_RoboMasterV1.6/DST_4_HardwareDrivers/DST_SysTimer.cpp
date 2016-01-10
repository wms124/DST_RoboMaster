/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_SysTimer.h"

//累积增加的systime定时计数
volatile uint32_t DST_SysTimer::sysTickUptime = 0;

DST_SysTimer systime;

DST_SysTimer::DST_SysTimer()
{
}

void DST_SysTimer::SysTickTimerInit(void)
{
	uint32_t ticks_us;
	RCC_ClocksTypeDef clocks;
	
	RCC_GetClocksFreq(&clocks);
	ticks_us = (uint32_t)clocks.HCLK_Frequency / 1000; //168M / 1000 = 168000
	ticks_us /= 8;	//168000 / 8 = 21000
	/*The function initializes the System Timer and its interrupt, and starts the System Tick Timer*/
	SysTick_Config(ticks_us);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//HCLK 8 div	
}

//滴答定时中断
void DST_SysTimer::SysTick_IRQ(void)
{
	sysTickUptime++;
	sys_1ms++;
	sys_2ms++;
	sys_3ms++;
	sys_5ms++;
	sys_7ms++;
	sys_10ms++;
	sys_20ms++;
	sys_50ms++;
	sys_500ms++;
}

//得到当前时间
uint32_t DST_SysTimer::GetSysTime_us(void) 
{
	register uint32_t ms;
	u32 value;
	ms = sysTickUptime;
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

void DST_SysTimer::Delay_us(uint32_t us)
{
	uint32_t now = GetSysTime_us();
	while (GetSysTime_us() - now < us);
}

void DST_SysTimer::Delay_ms(uint32_t ms)
{
	while (ms--)
			Delay_us(1000);
}









