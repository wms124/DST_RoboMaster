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


//所有硬件初始化
void DST_Hardware::DST_HardwareInit(void)
{
	//初始化中断
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
	//系统滴答定时器初始化
	systime.SysTickTimerInit();
	
	//MPU6050软件IIC初始化
	iic.IIC_Soft_Init();
	
	//MPU6050加速度陀螺仪初始化
	mpu6050.MPU6050_Init(42);
	
	//CAN1初始化
	can1.CAN_Configuration_Init();
	
	//CAN2初始化
	can2.CAN_Configuration_Init();
	
	//DBUS_USART2_DMA接收初始化
	dbus.USART2_DMA_Init();
	
	//PWM_OUT初始化
	pwmout.PWMOUT_Init(400);
	
	//LED初始化
	led.LED_Init();
}































