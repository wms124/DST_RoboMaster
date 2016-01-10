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
    LAST,		//��һ�ε�
    CURR,		//��ǰ��
    NEXT		//��һ�ε�
};

class DST_GimbleControl:public DST_PID{

public:
	
	DST_GimbleControl();

	/*
	 * λ�û�����
	 * @param pit_cur_l: Pit��ǰλ��
	 * @param pit_exp_l: Pit����λ��
	 * @param yaw_cur_l: Yaw��ǰλ��
	 * @param yaw_exp_l: Yaw����λ��
	 */
	void Location_Control(float pit_cur_l, float pit_exp_l,float yaw_cur_l, float yaw_exp_l);

private:
	
	//����ʽPID�������������أ�����ȶ���
	float pit_l_err[3];//PIT���
	float yaw_l_err[3];//YAW���
		
	float Pit_Control_value;	//Pit�����ֵ
	float Yaw_Control_value;	//Yaw�����ֵ

	/********6025���λ�û�PID********
	_6025_L_PID: 6025 Location PID
  */
	DST_PID _6025_L_PID;	

};
extern DST_GimbleControl gimble_control;






























#endif
