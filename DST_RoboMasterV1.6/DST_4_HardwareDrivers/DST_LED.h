#ifndef __DST_LED_H__
#define __DST_LED_H__
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



class DST_LED{

public:
	
	DST_LED();
	
	//LED初始化
	void LED_Init(void);

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
	void LED_Color_Set(uint8_t led_colour);

private:
	
};
extern DST_LED led;





















#endif

