#ifndef __DST_HARDWAREPINCONFIG_H__
#define __DST_HARDWAREPINCONFIG_H__
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


/****************************    COLOURFUL_LED Pin config   *******************************/
// D4 
#define RCC_COLOURFUL_LED_D4				RCC_AHB1Periph_GPIOD
#define GPIO_COLOURFUL_LED_D4				GPIOD
#define Pin_COLOURFUL_LED_D4_B			GPIO_Pin_3
#define Pin_COLOURFUL_LED_D4_R			GPIO_Pin_4
#define Pin_COLOURFUL_LED_D4_G			GPIO_Pin_7

// D5
#define RCC_COLOURFUL_LED_D5				RCC_AHB1Periph_GPIOE
#define GPIO_COLOURFUL_LED_D5				GPIOE
#define Pin_COLOURFUL_LED_D5_B			GPIO_Pin_7
#define Pin_COLOURFUL_LED_D5_R			GPIO_Pin_8
#define Pin_COLOURFUL_LED_D5_G			GPIO_Pin_10

#define Pin_D9				GPIO_Pin_1
#define Pin_D7				GPIO_Pin_0



/****************************    CAN Pin config   *******************************/
//CAN_1
#define RCC_CAN_1						RCC_AHB1Periph_GPIOD
#define GPIO_CAN_1					GPIOD
#define CAN_1_TX						GPIO_Pin_1
#define CAN_1_RX						GPIO_Pin_0

//CAN_2
#define RCC_CAN_2						RCC_AHB1Periph_GPIOB
#define GPIO_CAN_2					GPIOB
#define CAN_2_TX						GPIO_Pin_13
#define CAN_2_RX						GPIO_Pin_12


/****************************    IIC Pin config   *******************************/
#define RCC_IIC				RCC_AHB1Periph_GPIOB
#define GPIO_IIC			GPIOB
#define Pin_IIC_SCL		GPIO_Pin_6
#define Pin_IIC_SDA 	GPIO_Pin_7

#define SCL_H         GPIO_IIC->BSRRL = Pin_IIC_SCL
#define SCL_L         GPIO_IIC->BSRRH = Pin_IIC_SCL
#define SDA_H         GPIO_IIC->BSRRL = Pin_IIC_SDA
#define SDA_L         GPIO_IIC->BSRRH = Pin_IIC_SDA
#define SCL_R   		  GPIO_IIC->IDR   & Pin_IIC_SCL
#define SDA_R		      GPIO_IIC->IDR   & Pin_IIC_SDA



#endif









