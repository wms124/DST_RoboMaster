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
	/********************  ���캯��  ***********************/
	//
	//////////////////////////////////////////////////////////
	// roll=0, pitch=0, yaw=0 ʱ q0 = 1, q1 =0, q2 = 0, q3 = 0
	Quaternion(){
		q0 = 1; q1 = q2 = q3 = 0;
	}
	//��ֵ���캯��
	Quaternion(const float _q0, const float _q1, const float _q2, const float _q3){
		q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3;
	}
	//���ء�()�������
	void operator () (const float _q0, const float _q1, const float _q2, const float _q3){
			q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3;
	}
	
	//�Ƕ�ת����
	float radians(float deg);
	//����ת�Ƕ�
	float degrees(float rad);
	//��֤����ֵ��Ч
	float safe_asin(float v);
	
	//���ظ���Ԫ���ĵ�Ч��ת�����е���������
  void Vector_G(Vector3 &V_g);
	
	//һ�����������������Ԫ��
	void Runge_Kutta_1st(const Vector3 &w, float deltaT);
	
	//��Ԫ����һ��
	void Q_normalize(void);
	
	//��Ԫ��תŷ����
  void Q_Euler(float *roll, float *pitch, float *yaw);
	
private:
	

};
extern Quaternion Q;












#endif








