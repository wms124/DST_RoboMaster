#ifndef __DST_MPU6050_H__
#define __DST_MPU6050_H__
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
#include "DST_Vector3.h"



#define CALIBRATING_GYRO_CYCLES             200
#define CALIBRATING_ACC_CYCLES              200

#define ACC_1G 			4096		//�ɼ��ٶȼƵ�����ȷ��


#define EE_6050_ACC_X_OFFSET_ADDR	1
#define EE_6050_ACC_Y_OFFSET_ADDR	2
#define EE_6050_ACC_Z_OFFSET_ADDR	3
#define EE_6050_GYRO_X_OFFSET_ADDR	4
#define EE_6050_GYRO_Y_OFFSET_ADDR	5
#define EE_6050_GYRO_Z_OFFSET_ADDR	6

class DST_MPU6050 : public DST_IIC_Soft{

public:

	bool Calibrated_ACC, Calibrated_GYRO;
	int16_t acc_temp[3];
	int16_t gyro_temp[3];
	Vector3 ACC_Offset,GYRO_Offset;
	
	DST_MPU6050();
	void MPU6050_Pin_Config();
	void MPU6050_Init(uint16_t lpf);

	//���ٶȲ���
	void Read_ACC_Data(void);
	//���ٶȲ���
	void Read_GYRO_Data(void);

	//���ؼ��ٶ�ADCֵ
	Vector3 Get_ACC_ADC(void);
	//����������ADCֵ
	Vector3 Get_GYRO_ADC(void);
	
	//У׼ACC
	void Calibration_ACC(void);
	//У׼GYRO
	void Calibration_GYRO(void);

	//���ص�λΪ��ÿ��Ľ��ٶ�
	Vector3 Get_GYRO_in_dps(void);
	
private:

	uint8_t mpu6050_buffer[14]; //�������ݻ�����

	Vector3 ACC_ADC, GYRO_ADC, GYRO_dps;

};
extern DST_MPU6050 mpu6050;




#endif


