#ifndef __DST_SYSTIMER_H__
#define __DST_SYSTIMER_H__

/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_RoboMasterInclude.h"



#define TICK_US	1000

class DST_SysTimer{
	
public:
		
	static volatile uint32_t sysTickUptime;
	uint16_t sys_1ms,sys_2ms,sys_3ms,sys_5ms,sys_7ms,sys_10ms,sys_20ms,sys_50ms,sys_500ms;

	DST_SysTimer();

	//系统滴答定时器初始化
	void SysTickTimerInit(void);

	//得到当前时间
	uint32_t GetSysTime_us(void);

	//滴答定时器中断调用函数
	void SysTick_IRQ(void);

	//延时us
	void Delay_us(uint32_t us);

	//延时ms
	void Delay_ms(uint32_t ms);

private:

};
extern DST_SysTimer systime;



















#endif
