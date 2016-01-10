/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_Quaternion.h"
#include "DST_Vector3.h"

Quaternion Q;


//角度转弧度
float Quaternion::radians(float deg) {
	return deg * DEG_TO_RAD;
}
//弧度转角度
float Quaternion::degrees(float rad) {
	return rad * RAD_TO_DEG;
}

//保证输入值是有效的
float Quaternion::safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}

//四元数归一化
void Quaternion::Vector_G(Vector3 &V_g)
{
	V_g.x = 		2*(q1*q3 - q0*q2);
	V_g.y = 		2*(q0*q1 + q2*q3);
	V_g.z = 1 - 2*(q1*q1 + q2*q2);
}


//一阶龙格库塔法更新四元数
void Quaternion::Runge_Kutta_1st(const Vector3 &w, float deltaT)
{
	q0 = q0 + 0.5f*deltaT * (-w.x*q1 - w.y*q2 - w.z*q3);
	q1 = q1 + 0.5f*deltaT * ( w.x*q0 - w.y*q3 + w.z*q2);
	q2 = q2 + 0.5f*deltaT * ( w.x*q3 + w.y*q0 - w.z*q1);
	q3 = q3 + 0.5f*deltaT * (-w.x*q2 + w.y*q1 + w.z*q0);
}


//四元数归一化
void Quaternion::Q_normalize(void)
{
	float length = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= length;
	q1 *= length;
	q2 *= length;
	q3 *= length;
}


//四元数转换欧拉角
void Quaternion::Q_Euler(float *roll, float *pitch, float *yaw)
{
//	*roll  = degrees(atan2f(2.0f*(q1*q3 + q0*q2),1 - 2.0f*(q0*q0 + q3*q3)));
//	*pitch = degrees(safe_asin(2.0f*(q2*q3 - q0*q1)));
//	*yaw   = degrees(atan2f(2.0f*(q1*q2 - q0*q3),1 - 2.0f*(q1*q1 + q2*q2)));
	if(roll){
		*roll  = degrees(_math.fast_atan2(2.0f*(q0*q1 + q2*q3),1 - 2.0f*(q1*q1 + q2*q2)));
	}
	if(pitch){
		*pitch = degrees(safe_asin(2.0f*(q0*q2 - q1*q3)));
	}
	if(yaw){
		//*yaw   = _math.To_180_degrees(_math.fast_atan2(2.0f*(q1*q2 - q0*q3),2.0f*(q0*q0 + q1*q1) - 1) * 57.3f + imu.yaw_correct);
	}
}


















