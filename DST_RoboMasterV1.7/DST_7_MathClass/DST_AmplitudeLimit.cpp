/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/
#include "DST_AmplitudeLimit.h"




DST_AmplitudeLimit limit;

DST_AmplitudeLimit::DST_AmplitudeLimit()
{

}

/****************    uin16_t Amplitudelimit    ******************/
uint16_t DST_AmplitudeLimit::LimitRange_uint16(uint16_t data, uint16_t min, uint16_t max)
{
	if(data < min)
		data = min;
	if(data > max)
		data = max;
	return data;
}

/****************    in16_t Amplitudelimit    *******************/
int16_t DST_AmplitudeLimit::LimitRange_int16(int16_t data, int16_t min, int16_t max)
{
	if(data < min)
		data = min;
	if(data > max)
		data = max;
	return data;
}

/***************    uint32_t Amplitudelimit    ******************/
uint32_t DST_AmplitudeLimit::LimitRange_uint32(uint32_t data, uint32_t min, uint32_t max)
{
	if(data < min)
		data = min;
	if(data > max)
		data = max;
	return data;
}

/***************    int32_t Amplitudelimit    *******************/
int32_t DST_AmplitudeLimit::LimitRange_int32(int32_t data, int32_t min, int32_t max)
{
	if(data < min)
		data = min;
	if(data > max)
		data = max;
	return data;
}

/***************    float Amplitudelimit    *******************/
float DST_AmplitudeLimit::LimitRange_float(float data, float min, float max)
{
	if(data < min)
		data = min;
	if(data > max)
		data = max;
	return data;
}



