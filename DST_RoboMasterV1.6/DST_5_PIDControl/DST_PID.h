#ifndef __DST_PID_H__
#define __DST_PID_H__
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


class DST_PID{

public:
	
	struct PID{
		float P;
		float I;
		float D;
	}_pid;
	
	DST_PID();
			
	//重载()运算符进行赋值
	void operator () (const float _p, const float _i, const float _d)
	{
		_pid.P=_p; _pid.I=_i; _pid.D=_d;
	}

private:
	

};

































#endif














