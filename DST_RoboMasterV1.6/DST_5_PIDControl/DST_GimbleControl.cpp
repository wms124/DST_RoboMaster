/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/
#include "DST_GimbleControl.h"


DST_GimbleControl gimble_control;


DST_GimbleControl::DST_GimbleControl()
{
	pit_l_err[0] = 0;pit_l_err[1] = 0;pit_l_err[2] = 0;//pitch轴位置误差初始化
	yaw_l_err[0] = 0;yaw_l_err[1] = 0;yaw_l_err[2] = 0;//yaw轴位置误差初始化0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000                            00 0
	/* 初始化6025电机位置环PID
	 * P->0.05; I->0.008; D->0
	*/
	_6025_L_PID(0.05,0.008,0);
}



/*
 * 位置环控制
 * @param pit_cur_l: Pit当前位置
 * @param pit_exp_l: Pit期望位置
 * @param yaw_cur_l: Yaw当前位置
 * @param yaw_exp_l: Yaw期望位置
 */
void DST_GimbleControl::Location_Control(float pit_cur_l, float pit_exp_l, float yaw_cur_l, float yaw_exp_l)
{
	//误差 = 期望 - 当前
	pit_l_err[0] = pit_exp_l - pit_cur_l;
	yaw_l_err[0] = yaw_exp_l - yaw_cur_l;
	if(ABS(pit_l_err[0]) > 10)
	{
		//PIT轴 PI控制
		Pit_Control_value += _6025_L_PID._pid.P * (pit_l_err[0]-pit_l_err[1]) + _6025_L_PID._pid.I * (pit_l_err[2] - 2*pit_l_err[1] + pit_l_err[0]);
	}
	if(ABS(yaw_l_err[0]) > 10)
	{
		//YAW轴 PI控制
		Yaw_Control_value += _6025_L_PID._pid.P * (yaw_l_err[0]-yaw_l_err[1]) + _6025_L_PID._pid.I * (yaw_l_err[2] - 2*yaw_l_err[1] + yaw_l_err[0]);
	}
	pit_l_err[2] = pit_l_err[1];
	pit_l_err[1] = pit_l_err[0];
	
	yaw_l_err[2] = yaw_l_err[1];
	yaw_l_err[1] = yaw_l_err[0];
	limit.LimitRange_float(Pit_Control_value,EC_VALUE_MIN,EC_VALUE_MAX);
	limit.LimitRange_float(Yaw_Control_value,EC_VALUE_MIN,EC_VALUE_MAX);
	can1.Send_CAN1_CMD(Pit_Control_value, 0, Yaw_Control_value);
}























