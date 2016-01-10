#ifndef __DST_GIMBLECONTROL_H__
#define __DST_GIMBLECONTROL_H__
/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_PID.h"
#include "DST_RoboMasterInclude.h"



enum {
    LAST,		//上一次的
    CURR,		//当前的
    NEXT		//下一次的
};

class DST_GimbleControl:public DST_PID{

public:
	
	DST_GimbleControl();

	/*
	 * 位置环控制
	 * @param pit_cur_l: Pit当前位置
	 * @param pit_exp_l: Pit期望位置
	 * @param yaw_cur_l: Yaw当前位置
	 * @param yaw_exp_l: Yaw期望位置
	 */
	void Location_Control(float pit_cur_l, float pit_exp_l,float yaw_cur_l, float yaw_exp_l);

private:
	
	//增量式PID，与三次误差相关，提高稳定性
	float pit_l_err[3];//PIT误差
	float yaw_l_err[3];//YAW误差
		
	float Pit_Control_value;	//Pit轴控制值
	float Yaw_Control_value;	//Yaw轴控制值

	/********6025电机位置环PID********
	_6025_L_PID: 6025 Location PID
  */
	DST_PID _6025_L_PID;	

};
extern DST_GimbleControl gimble_control;






























#endif
