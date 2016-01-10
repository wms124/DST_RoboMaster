#ifndef __DST_QUATERNION_H__
#define __DST_QUATERNION_H__
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

#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

class Quaternion{

public:
	
	float q0, q1, q2, q3;

	//////////////////////////////////////////////////////////
	//
	/********************  构造函数  ***********************/
	//
	//////////////////////////////////////////////////////////
	// roll=0, pitch=0, yaw=0 时 q0 = 1, q1 =0, q2 = 0, q3 = 0
	Quaternion(){
		q0 = 1; q1 = q2 = q3 = 0;
	}
	//赋值构造函数
	Quaternion(const float _q0, const float _q1, const float _q2, const float _q3){
		q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3;
	}
	//重载“()”运算符
	void operator () (const float _q0, const float _q1, const float _q2, const float _q3){
			q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3;
	}
	
	//角度转弧度
	float radians(float deg);
	//弧度转角度
	float degrees(float rad);
	//保证输入值有效
	float safe_asin(float v);
	
	//返回该四元数的等效旋转矩阵中的重力分量
  void Vector_G(Vector3 &V_g);
	
	//一阶龙格库塔法更新四元数
	void Runge_Kutta_1st(const Vector3 &w, float deltaT);
	
	//四元数归一化
	void Q_normalize(void);
	
	//四元数转欧拉角
  void Q_Euler(float *roll, float *pitch, float *yaw);
	
private:
	

};
extern Quaternion Q;












#endif








