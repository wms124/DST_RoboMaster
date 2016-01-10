#ifndef __DST_PWMOUT_H__
#define __DST_PWMOUT_H__
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


//#define INIT_DUTY 4000 	//u16(1000/0.25)
//#define ACCURACY 10000 	//u16(2500/0.25) //accuracy
//#define PWM_RADIO 4			//(8000 - 4000)/1000.0


#define MAXMOTORS 4

class DST_PWMOUT{

public:
	
	DST_PWMOUT();
	void PWMOUT_Init(uint16_t hz);

	void Set_PWMOUT(uint16_t *pwm_Out);

	////////////////////////////////////
	//设置PWM波范围：1000~2000;
	//1000最小
	//2000最大
	void SetPwm(uint16_t pwm[MAXMOTORS]);

private:
	
};
extern DST_PWMOUT pwmout;

















#endif







