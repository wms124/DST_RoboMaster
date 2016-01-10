/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/
#include "DST_PWMOUT.h"





DST_PWMOUT pwmout;

DST_PWMOUT::DST_PWMOUT()
{

}


void DST_PWMOUT::PWMOUT_Init(uint16_t hz)//400Hz
{
	uint16_t PrescalerValue = 0;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef 			TIM_OCInitStructure;
	GPIO_InitTypeDef				GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	PrescalerValue = (uint16_t)  ( SystemCoreClock / (hz * 1000)) - 1;
	
	TIM_TimeBaseStructure.TIM_Period = 999;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	//PWM_OUT1 -- TIM1_CH1 Init
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	//PWM_OUT2 -- TIM1_CH2 Init
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	//PWM_OUT3 -- TIM1_CH3 Init
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	//PWM_OUT4 -- TIM1_CH4 Init
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);//TIMx: where x can be 1 or 8 to select the TIMx peripheral.
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}


void DST_PWMOUT::Set_PWMOUT(uint16_t *pwm_Out)
{
	for(u8 i=0;i<4;i++)
	{
		if(pwm_Out[i] < 1000)
			pwm_Out[i] = 1000;
		else if(pwm_Out[i] > 2000)
			pwm_Out[i] = 2000;
	}
	TIM1->CCR1 = pwm_Out[0] - 1000; 
	TIM1->CCR2 = pwm_Out[1] - 1000;	
	TIM1->CCR3 = pwm_Out[2] - 1000; 
	TIM1->CCR4 = pwm_Out[3] - 1000;
}

////////////////////////////////////
//设置PWM波范围：1000~2000;
//1000最小
//2000最大
void DST_PWMOUT::SetPwm(uint16_t pwm[MAXMOTORS])
{
	for(u8 i=0;i<MAXMOTORS;i++)
	{
		if(pwm[i] < 1000)
			pwm[i] = 1000;
		else if(pwm[i] > 2000)
			pwm[i] = 2000;
	}
	
	TIM1->CCR1 = pwm[0] - 1000; 
	TIM1->CCR2 = pwm[1] - 1000;	
	TIM1->CCR3 = pwm[2] - 1000;
	TIM1->CCR4 = pwm[3] - 1000;
}







