#ifndef __DST_AMPLITUDELIMIT_H__
#define __DST_AMPLITUDELIMIT_H__
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


class DST_AmplitudeLimit{

public:
	
	DST_AmplitudeLimit();

	/****************    uin16_t Amplitudelimit    ******************/
	uint16_t LimitRange_uint16(uint16_t data, uint16_t min, uint16_t max);
	/****************    in16_t Amplitudelimit    *******************/
	int16_t LimitRange_int16(int16_t data, int16_t min, int16_t max);
	/***************    uint32_t Amplitudelimit    ******************/
	uint32_t LimitRange_uint32(uint32_t data, uint32_t min, uint32_t max);
	/***************    int32_t Amplitudelimit    *******************/
	int32_t LimitRange_int32(int32_t data, int32_t min, int32_t max);
	/***************    float Amplitudelimit    *******************/
	float LimitRange_float(float data, float min, float max);
	
private:
	
};
extern DST_AmplitudeLimit limit;









#endif








