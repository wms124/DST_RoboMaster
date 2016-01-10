#ifndef __DST_VECTOR3_H__
#define __DST_VECTOR3_H__
/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_BatQuadRotorAircraft
//		DD	 DD         SSS      TT						燕园开源四旋翼飞行器
//		DDDDD       SSSSSS       TT						作者：Dstone		
========================================================================================*/
/****************************************************************************************/
#include <math.h>



class Vector3{

public:
	
	float x,y,z;
	
	//////////////////////////////////////////////////////////
	//
	/********************  构造函数  ***********************/
	//
	//////////////////////////////////////////////////////////
	//默认构造函数
	Vector3(){
		x = y = z = 0.0f;
	}
	//复制构造函数
	Vector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z){}
	//带参数构造函数
	Vector3(float vx, float vy, float vz) : x(vx), y(vy), z(vz){}
	//重载赋值运算，返回引用，实现左值
	Vector3 &operator = (const Vector3 &v){
		x=v.x;
		y=v.y;
		z=v.z;
		return *this;
	}
	//////////////////////////////////////////////////////////
	//
	/********************  向量运算  ***********************/
	//
	//////////////////////////////////////////////////////////
	void operator () (const float vx, const float vy, const float vz)
	{
			x= vx; y= vy; z= vz;
	}
	//重载“==”，“!=”运算符
	bool operator == (const Vector3 &v) const{
		return x==v.x && y==v.y && z==v.z;
	}
	bool operator != (const Vector3 &v) const{
		return x!=v.x || y!=v.y || z!=v.z;
	}
	//置为零向量
	void zero(){
		x = y = z = 0.0f;
	}
	//重载“-”运算符
	Vector3 operator - () const{
		return Vector3(-x,-y,-z);
	}
	//重载二元“+”和“-”运算符
	Vector3 operator + (const Vector3 &v) const{
		return Vector3(x+v.x, y+v.y, z+v.z);
	}
	Vector3 operator - (const Vector3 &v) const{
		return Vector3(x-v.x, y-v.y, z-v.z);
	}
	//与标量的乘、除法
	Vector3 operator * (float a) const{
		return Vector3(x*a, y*a, z*a);
	}
	Vector3 operator / (float a) const{
		float oneOverA = 1.0f / a;	//a不能为0
		return Vector3(x*oneOverA, y*oneOverA, z*oneOverA);
	}
	//重载自反运算符
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
		float oneOverA = 1.0f / a;	//a不能为0
		x *= oneOverA; y *= oneOverA; z *= oneOverA;
		return *this;
	}
	//向量标准化
	void normalize(){
		float magSq = x*x + y*y + z*z;
		if(magSq > 0.0f){ //检查除0
			float oneOverMag = 1.0f / sqrtf(magSq);
			x *= oneOverMag;
			y *= oneOverMag;
			z *= oneOverMag;
		}
	}
	//向量点乘
	float operator * (const Vector3 &v) const {
		return x*v.x + y*v.y + z*v.z;
	}
};

//////////////////////////////////////////////////////////
//
/********************  非成员函数 ***********************/
//
//////////////////////////////////////////////////////////
//求向量模
inline float VectorMag(const Vector3 &v){
	return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

//向量叉乘
inline Vector3 crossProduct(const Vector3 &v1, const Vector3 &v2){
	return Vector3(
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	);
}

//实现标量左乘
inline Vector3 operator * (float a, const Vector3 &v){
	return Vector3(a*v.x, a*v.y, a*v.z);
}

//计算两点间距离
inline float distance(const Vector3 &a, const Vector3 &b){
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrtf(dx*dx + dy*dy + dz*dz);
}







#endif




