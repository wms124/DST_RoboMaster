#ifndef __DST_ROBOMASTERCONFIG_H__
#define __DST_ROBOMASTERCONFIG_H__
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



#define ARMAPI	extern "C"


/****************************    TRUE or FALSE   *******************************/
#define _TRUE			1
#define _FALSE		0

/*****************************   START or STOP   *******************************/
#define _START		1
#define _STOP			0

/*****************************   OK && Error   *******************************/
#define _OK							0
#define _Error					1

/*****************************    && Error   *******************************/
#define _SUCCEED				0
#define _DEFEATED				1

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
#define LED_D4_COLOUR_BLUE			0x01
#define	LED_D4_COLOUR_RED				0x02
#define	LED_D4_COLOUR_GREEN			0x04
#define LED_D5_COLOUR_BLUE			0x08
#define LED_D5_COLOUR_RED 			0x10
#define LED_D5_COLOUR_GREEN			0x20
//#define 
//#define 
#define LED_ALLOON							0xFF
#define LED_ALLOFF							0x00


/***************************************************/
//HG900最大限制值
/***************************************************/
#define EC_VALUE_MAX			5000		//electric current (-5000~5000)
#define EC_VALUE_MIN		 -5000		//electric current (-5000~5000)
#define ANGLE_VALUE_MAX		8191		//mechanical angle value (0~8191)


/***************************************************/
//6025云台电机CAN接收ID
/***************************************************/
#define PITCH_ID			0x201
#define YAW_ID				0x203



/***************************************************/
//EC60云台电机CAN接收ID
/***************************************************/
#define _1_ID				0x201
#define _2_ID				0x202
#define _3_ID				0x203
#define _4_ID				0x204






























#endif


