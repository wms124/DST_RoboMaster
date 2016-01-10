/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_LED.h"


DST_LED led;

DST_LED::DST_LED()
{

}

//LED初始化
void DST_LED::LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_COLOURFUL_LED_D4,ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin = Pin_COLOURFUL_LED_D4_B | Pin_COLOURFUL_LED_D4_R | Pin_COLOURFUL_LED_D4_G;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	GPIO_SetBits(GPIO_COLOURFUL_LED_D4, GPIO_Pin_3);		
	GPIO_SetBits(GPIO_COLOURFUL_LED_D4, GPIO_Pin_4);		
	GPIO_SetBits(GPIO_COLOURFUL_LED_D4, GPIO_Pin_7);			
	
	RCC_AHB1PeriphClockCmd(RCC_COLOURFUL_LED_D5,ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin = Pin_COLOURFUL_LED_D5_B | Pin_COLOURFUL_LED_D5_R | Pin_COLOURFUL_LED_D5_G | Pin_D7 | Pin_D9;
	GPIO_Init(GPIO_COLOURFUL_LED_D5,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIO_COLOURFUL_LED_D5, Pin_COLOURFUL_LED_D5_B);		
	GPIO_SetBits(GPIO_COLOURFUL_LED_D5, Pin_COLOURFUL_LED_D5_R);		
	GPIO_SetBits(GPIO_COLOURFUL_LED_D5, Pin_COLOURFUL_LED_D5_G);			
	
	GPIO_SetBits(GPIOE, Pin_D7);			
	GPIO_SetBits(GPIOE, Pin_D9);			
}


//多彩LED颜色设置
//Freewill LED set use #define
//For example param: LED_COLOUR_BLUE | LED_COLOUR_RED
/***************************************************/
//           |        D5       |       D4
//-----------|-----------------|-----------------
//     |     |LED_G|LED_R|LED_B|LED_G|LED_R|LED_B
//  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0
//-----------------------------------------------
//#define LED_D4_COLOUR_BLUE			0x01
//#define	LED_D4_COLOUR_RED				0x02
//#define	LED_D4_COLOUR_GREEN			0x04
//#define LED_D5_COLOUR_BLUE			0x08
//#define LED_D5_COLOUR_RED 			0x10
//#define LED_D5_COLOUR_GREEN			0x20
//#define 
//#define 
//#define LED_ALLOON							0xFF
//#define LED_ALLOFF							0x00
/***************************************************/
void DST_LED::LED_Color_Set(uint8_t led_colour)
{
	uint8_t i;
	uint8_t led_set_colour = led_colour;
	
	if(led_colour&0x07)
	{
		for(i=0;i<3;i++)
		{
			if(led_colour&0x01)
				GPIO_ResetBits(GPIO_COLOURFUL_LED_D4,0x0001<<(6+i));
			else
				GPIO_SetBits(GPIO_COLOURFUL_LED_D4,0x0001<<(6+i));
			led_colour >>= 1;
		}
	}
	if(led_set_colour&0x38)
	{
		for(i=0;i<3;i++)
		{
			if(led_set_colour&0x01)
				GPIO_ResetBits(GPIO_COLOURFUL_LED_D5,0x0001<<(6+i));
			else
				GPIO_SetBits(GPIO_COLOURFUL_LED_D5,0x0001<<(6+i));
			led_set_colour >>= 1;
		}
	}
}






























