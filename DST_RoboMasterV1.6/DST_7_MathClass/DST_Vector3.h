#ifndef __DST_VECTOR3_H__
#define __DST_VECTOR3_H__
/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_BatQuadRotorAircraft
//		DD	 DD         SSS      TT						��԰��Դ�����������
//		DDDDD       SSSSSS       TT						���ߣ�Dstone		
========================================================================================*/
/****************************************************************************************/
#include <math.h>



class Vector3{

public:
	
	float x,y,z;
	
	//////////////////////////////////////////////////////////
	//
	/********************  ���캯��  ***********************/
	//
	//////////////////////////////////////////////////////////
	//Ĭ�Ϲ��캯��
	Vector3(){
		x = y = z = 0.0f;
	}
	//���ƹ��캯��
	Vector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z){}
	//���������캯��
	Vector3(float vx, float vy, float vz) : x(vx), y(vy), z(vz){}
	//���ظ�ֵ���㣬�������ã�ʵ����ֵ
	Vector3 &operator = (const Vector3 &v){
		x=v.x;
		y=v.y;
		z=v.z;
		return *this;
	}
	//////////////////////////////////////////////////////////
	//
	/********************  ��������  ***********************/
	//
	//////////////////////////////////////////////////////////
	void operator () (const float vx, const float vy, const float vz)
	{
			x= vx; y= vy; z= vz;
	}
	//���ء�==������!=�������
	bool operator == (const Vector3 &v) const{
		return x==v.x && y==v.y && z==v.z;
	}
	bool operator != (const Vector3 &v) const{
		return x!=v.x || y!=v.y || z!=v.z;
	}
	//��Ϊ������
	void zero(){
		x = y = z = 0.0f;
	}
	//���ء�-�������
	Vector3 operator - () const{
		return Vector3(-x,-y,-z);
	}
	//���ض�Ԫ��+���͡�-�������
	Vector3 operator + (const Vector3 &v) const{
		return Vector3(x+v.x, y+v.y, z+v.z);
	}
	Vector3 operator - (const Vector3 &v) const{
		return Vector3(x-v.x, y-v.y, z-v.z);
	}
	//������ĳˡ�����
	Vector3 operator * (float a) const{
		return Vector3(x*a, y*a, z*a);
	}
	Vector3 operator / (float a) const{
		float oneOverA = 1.0f / a;	//a����Ϊ0
		return Vector3(x*oneOverA, y*oneOverA, z*oneOverA);
	}
	//�����Է������
	Vector3 operator += (const Vector3 &v){
		x += v.x; y += v.y; z += v.z;
		return *this;
	}
	Vector3 operator -= (const Vector3 &v){
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}
	Vector3 operator *= (float a){
		x *= a; y *= a; z *= a;
		return *this;
	}
	Vector3 operator /= (float a){
		float oneOverA = 1.0f / a;	//a����Ϊ0
		x *= oneOverA; y *= oneOverA; z *= oneOverA;
		return *this;
	}
	//������׼��
	void normalize(){
		float magSq = x*x + y*y + z*z;
		if(magSq > 0.0f){ //����0
			float oneOverMag = 1.0f / sqrtf(magSq);
			x *= oneOverMag;
			y *= oneOverMag;
			z *= oneOverMag;
		}
	}
	//�������
	float operator * (const Vector3 &v) const {
		return x*v.x + y*v.y + z*v.z;
	}
};

//////////////////////////////////////////////////////////
//
/********************  �ǳ�Ա���� ***********************/
//
//////////////////////////////////////////////////////////
//������ģ
inline float VectorMag(const Vector3 &v){
	return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

//�������
inline Vector3 crossProduct(const Vector3 &v1, const Vector3 &v2){
	return Vector3(
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	);
}

//ʵ�ֱ������
inline Vector3 operator * (float a, const Vector3 &v){
	return Vector3(a*v.x, a*v.y, a*v.z);
}

//������������
inline float distance(const Vector3 &a, const Vector3 &b){
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrtf(dx*dx + dy*dy + dz*dz);
}







#endif




