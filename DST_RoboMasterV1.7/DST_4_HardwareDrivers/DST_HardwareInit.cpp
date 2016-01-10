/****************************************************************************************/
/*========================================================================================
//		DDDDD       SSSSSS	  TTTTTTTT      
//		DD   DD    SSS					 TT						
//		DD   DD		   SSSS        TT 					DST_RoboMaster
//		DD	 DD         SSS      TT						YSU_RoboHunter && YSU_RoboBlade
//		DDDDD       SSSSSS       TT						Author:Dstone		
========================================================================================*/
/****************************************************************************************/

#include "DST_HardwareInit.h"


DST_Hardware hardware;

DST_Hardware::DST_Hardware()
{

}


//����Ӳ����ʼ��
void DST_Hardware::DST_HardwareInit(void)
{
	//��ʼ���ж�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
	//ϵͳ�δ�ʱ����ʼ��
	systime.SysTickTimerInit();
	
	//MPU6050���IIC��ʼ��
	iic.IIC_Soft_Init();
	
	//MPU6050���ٶ������ǳ�ʼ��
	mpu6050.MPU6050_Init(42);
	
	//CAN1��ʼ��
	can1.CAN_Configuration_Init();
	
	//CAN2��ʼ��
	can2.CAN_Configuration_Init();
	
	//DBUS_USART2_DMA���ճ�ʼ��
	dbus.USART2_DMA_Init();
	
	//PWM_OUT��ʼ��
	pwmout.PWMOUT_Init(400);
	
	//LED��ʼ��
	led.LED_Init();
}































